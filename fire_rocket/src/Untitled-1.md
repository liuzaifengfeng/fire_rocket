这段代码是一套运行在 **ESP32-C3** 上的**遥测与飞行数据记录固件（Flight Recorder / Telemetry Monitor）**。

---

### 一、 现有代码架构深度解析

```
       [10 轴串口传感器 (UART 3/4)]
                     │ 字节流 (115200)
                     ▼
         acceptByte() / parser.feed()
                     │ 解包出完整单帧 Sample
                     ▼
   ┌─────────────────┴──────────────────┐
   ▼                                    ▼
monitor.accept()                 recorder.append()
(零点校准/姿态/相对高度)         (落盘记录/Flash/SD)

```

1. **功能定位与安全限制**
* **纯监控固件**：顶部明确标有 `Telemetry-only firmware`，且在 `setup()` 中将 GPIO 0、1、7 强行拉低（`disabledOutputs`），锁死了物理点火和开伞舵机等执行器接口。
* **状态机与重放机制**：支持真实传感器在线工作（`LIVE`）和通过 USB 串口灌入 16 进制报文的离线仿真测试（`REPLAY`）。


2. **核心模块职责**
* `parser`（解析器）：在 `acceptByte()` 中接收串口字节流，负责帧头比对、协议校验和数据包反序列化。
* `monitor`（状态监控器）：维护飞行基准（`zeroHeight` 归零基准面）、校验数据新鲜度（`fresh()`）、解算相对起飞点高度（`relativeHeight()`）和机体倾角（`tilt()`）。
* `recorder`（黑匣子记录器）：异步缓存并持久化飞行数据，具有导出（`EXPORT`）与格式化（`FORMAT`）功能。
* `display`（OLED 显示）：使用 U8g2 驱动 72x40 分辨率 OLED，轮询分屏显示实时高度/倾角、通信质量和录制状态。


3. **主循环调度特点**
* 采用**时间片单循环（Super-Loop）**，无阻塞等待。`sensor.read()` 配有单次 512 字节的消费预算，保障了高波特率下的接收效率。



---

### 二、 卡尔曼高度滤波的最佳接入点

在现有逻辑中，**最佳切入点是 `acceptByte()` 中 `if (updated)` 的触发分支**。
当 `parser.feed()` 完成一帧完整数据的校验与反序列化后，传感器数据（加速度、姿态四元数、气压高度）在此处同步就绪。

#### 数据提取逻辑：

1. **观测值（高度测量）**：直接使用 `monitor.relativeHeight()`。`monitor` 内部已处理完起飞地面的基准气压偏置，无需再重新计算 $P_0$。
2. **控制输入（垂直加速度）**：从 `parser.sample()` 中提取机体加速度与四元数，完成世界坐标系投影并剔除重力。

---

### 三、 改造实现方案

#### 1. 编写轻量级标量卡尔曼滤波类（直接嵌入文件顶部）

无需外部矩阵库，纯标量展开，非常适合没有硬件 FPU 的 ESP32-C3：

```cpp
class AltitudeKalman {
public:
    void reset(float initZ = 0.0f) {
        z = initZ;
        vz = 0.0f;
        p00 = 1.0f; p01 = 0.0f;
        p10 = 0.0f; p11 = 1.0f;
        lastUpdateUs = 0;
        initialized = true;
    }

    // dt 单位：秒； az 单位：m/s^2 (天向，已去重力)； z_baro 单位：米
    void update(float z_baro, float az, float dt) {
        if (!initialized || dt <= 0.0f || dt > 0.2f) return;

        // 1. 状态外推 (Prediction)
        float z_pred  = z + vz * dt + 0.5f * az * dt * dt;
        float vz_pred = vz + az * dt;

        // 2. 协方差外推
        // Q 过程噪声参数 (针对 20Hz 调校)
        const float Q_accel = 1.0f; // 加速度噪声方差
        float q00 = 0.25f * dt * dt * dt * dt * Q_accel;
        float q01 = 0.5f * dt * dt * dt * Q_accel;
        float q11 = dt * dt * Q_accel;

        float p00_p = p00 + dt * (p01 + p10) + dt * dt * p11 + q00;
        float p01_p = p01 + dt * p11 + q01;
        float p10_p = p10 + dt * p11 + q01;
        float p11_p = p11 + q11;

        // 3. 计算增益 (Measurement Update)
        const float R_baro = 0.25f; // 气压计方差 (可接受米级误差时取 0.25~0.5)
        float S = p00_p + R_baro;
        float K0 = p00_p / S;
        float K1 = p10_p / S;

        // 4. 残差纠偏
        float y = z_baro - z_pred;
        z  = z_pred  + K0 * y;
        vz = vz_pred + K1 * y;

        // 5. 协方差更新
        p00 = (1.0f - K0) * p00_p;
        p01 = (1.0f - K0) * p01_p;
        p10 = p10_p - K1 * p00_p;
        p11 = p11_p - K1 * p01_p;
    }

    float altitude() const { return z; }
    float velocity() const { return vz; }
    bool  ready()    const { return initialized; }

private:
    float z = 0.0f;
    float vz = 0.0f;
    float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
    uint32_t lastUpdateUs = 0;
    bool initialized = false;
};

```

---

#### 2. 在原有工程中挂载算法

##### A. 实例化对象并声明全局时间戳

在匿名命名空间 `namespace { ... }` 中添加：

```cpp
AltitudeKalman kf;
uint32_t lastKfUs = 0;

```

##### B. 在 `acceptByte()` 中执行解算

修改 `acceptByte()` 中的 `if (updated)` 段落：

```cpp
    if (updated) {
        monitor.accept(parser.sample(), settings);
        recorder.append(monitor, settings, now);

        // ================== 卡尔曼滤波插入点 ==================
        const auto& s = parser.sample();
        float relHeight = 0.0f;

        // 必须处于已归零且数据未过期状态
        if (monitor.zeroed() && monitor.relativeHeight(now, settings, relHeight)) {
            uint32_t currentUs = micros();
            if (!kf.ready()) {
                kf.reset(relHeight);
                lastKfUs = currentUs;
            } else {
                float dt = (currentUs - lastKfUs) * 1e-6f;
                lastKfUs = currentUs;

                // 提取机体三轴加速度与四元数，投影到世界坐标垂直天向
                // 注：若模块输出为标准四元数 q[0]=w, q[1]=x, q[2]=y, q[3]=z
                float q0 = s.quaternion[0];
                float q1 = s.quaternion[1];
                float q2 = s.quaternion[2];
                float q3 = s.quaternion[3];

                float ax = s.accel[0];
                float ay = s.accel[1];
                float az = s.accel[2];

                // 旋转矩阵第 3 行 * 机体加速度 - 重力 g
                float a_nav_z = 2.0f * (q1 * q3 - q0 * q2) * ax +
                                2.0f * (q2 * q3 + q0 * q1) * ay +
                                (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az - 9.80665f;

                // 若失重跌落出现异常微重力，限制加速度剧烈漂移
                float accNorm = sqrtf(ax * ax + ay * ay + az * az);
                if (accNorm < 1.96f) { // < 0.2g 时认为失重，暂停加速度输入
                    a_nav_z = 0.0f;
                }

                kf.update(relHeight, a_nav_z, dt);
            }
        } else {
            // 如果设备未归零，保持滤波器待命
            if (kf.ready()) kf.reset(0.0f);
        }
        // =====================================================
    }

```

##### C. 关联零点重置

当执行 `zeroHeight()` 成功时，同步复位滤波器：

```cpp
bool zeroHeight() {
    if (recorder.active() || recorder.exporting()) return false;
    bool ok = monitor.zero(millis(), settings);
    if (ok) kf.reset(0.0f); // 同步归零卡尔曼高度
    return ok;
}

```

##### D. 屏幕与串口输出呈现

在 `draw()` 的 `page == 0` 中，将滤波结果与垂直速度显示到小屏：

```cpp
    if (page == 0) {
        if (kf.ready()) {
            snprintf(line, sizeof(line), "KF%6.1fm", kf.altitude());
            display.drawStr(0, 7, line);
            snprintf(line, sizeof(line), "Vz%6.1fm/s", kf.velocity());
            display.drawStr(0, 15, line);
        } else {
            display.drawStr(0, 7, "KF --");
            display.drawStr(0, 15, "Vz --");
        }
        // ... 原有健康状态及 LIVE/REPLAY 显示保留

```

在 `status()` 函数内加入打印：

```cpp
    if (kf.ready()) {
        Serial.printf("KF_HEIGHT=%.3f KF_VEL=%.3f\n", kf.altitude(), kf.velocity());
    }

```

---

### 四、 针对此工程的落地细节

1. **四元数定义匹配**：
需确认 `TelemetryCore.h` 内 `sample.quaternion` 的顺序是 $[w, x, y, z]$ 还是 $[x, y, z, w]$。若顺序不同，调整投影公式中下标对应关系即可。若模块仅提供欧拉角，使用欧拉角投影旋转矩阵第三行即可。
2. **时间步长（$\Delta t$）计算**：
原系统运行在 20Hz 左右，但由于串口接收存在几十毫秒的不均匀抖动，务必使用 `micros()` 动态计算两帧之间的真实 `dt`，不可硬编码 `dt = 0.05f`。
3. **数据黑匣子持久化**：
后续若需要分析开伞高度曲线，可以在 `recorder.append()` 接口中扩展两个入参（`kf.altitude()` 与 `kf.velocity()`），一同落入 Flash 或 SD 卡中，便于飞行后导出 CSV 复盘。