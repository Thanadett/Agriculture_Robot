#include <Arduino.h>
#include "stepper.h"

// ================== Pin Mapping (แก้ให้ตรงฮาร์ดแวร์) ==================
#define PIN_STEP 17
#define PIN_DIR 16
#define PIN_ENABLE 4
#define STEP_DELAY_US 1200 // ยิ่งน้อย = เร็วขึ้น (เริ่ม conservatively)

// อยากใช้ Active-High ให้เปลี่ยนเป็น false ตรงนี้ (ค่าซ้ำกับใน header ได้ แต่กันพลาด)
// #undef ENABLE_INVERT
// #define ENABLE_INVERT false

UnifiedStepper Nema17(PIN_STEP, PIN_DIR, PIN_ENABLE, STEP_DELAY_US);

// โหมดเดโม: สลับ CW 3s -> หยุด 0.5s -> CCW 3s -> หยุด 0.5s (ถ้าไม่มีคำสั่งจาก Serial)
// ปิดเดโมได้โดยตั้ง DEMO_MODE = 0
#define DEMO_MODE 1

#if DEMO_MODE
enum Phase
{
    CW_RUN,
    PAUSE1,
    CCW_RUN,
    PAUSE2
};
Phase phase = CW_RUN;
unsigned long t0 = 0;

void enterPhase(Phase p)
{
    phase = p;
    t0 = millis();
    switch (phase)
    {
    case CW_RUN:
        Nema17.rotateContinuous(true);
        break;
    case CCW_RUN:
        Nema17.rotateContinuous(false);
        break;
    case PAUSE1:
    case PAUSE2:
        Nema17.stop();
        break;
    }
}
#endif

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 1000)
    {
        ;
    }

    Nema17.begin();

    // ตั้งความเร็วสูงสุด/ความเร่งเพิ่มเองได้
    // Nema17.setMaxSpeed(800);     // steps/s
    // Nema17.setAcceleration(1600);// steps/s^2

#if DEMO_MODE
    delay(200);
    enterPhase(CW_RUN);
#endif
}

void loop()
{
    // tick ต้องเรียกทุกลูปเพื่อให้ AccelStepper ทำงาน
    Nema17.tick();

    // --- รับคำสั่ง Serial แบบบรรทัด ---
    static String line;
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (line.length() > 0)
            {
                // ตัวอย่าง: "STP C_Up=DOWN", "STP C_Up=UP", "STP C_Dn=DOWN", "STP C_Dn=UP"
                if (!stepper_handle_line(line, Nema17))
                {
                    // คำสั่งอย่างอื่นสำหรับ debug
                    if (line.equalsIgnoreCase("CW"))
                        Nema17.rotateContinuous(true);
                    else if (line.equalsIgnoreCase("CCW"))
                        Nema17.rotateContinuous(false);
                    else if (line.equalsIgnoreCase("STOP"))
                        Nema17.stop();
                    else if (line.startsWith("MOVE "))
                    {
                        long s = line.substring(5).toInt();
                        Nema17.moveSteps(s);
                    }
                }
                line = "";
            }
        }
        else
        {
            line += c;
        }
    }

#if DEMO_MODE
    // เดินเดโมถ้าไม่มี input
    static unsigned long last_input_ms = 0;
    // ถ้าอยากให้ปิดเดโมทันทีที่มี Serial input สามารถตรวจจับและสั่ง stop() ได้

    const unsigned long now = millis();
    switch (phase)
    {
    case CW_RUN:
        if (now - t0 >= 3000)
            enterPhase(PAUSE1);
        break;
    case PAUSE1:
        if (now - t0 >= 500)
            enterPhase(CCW_RUN);
        break;
    case CCW_RUN:
        if (now - t0 >= 3000)
            enterPhase(PAUSE2);
        break;
    case PAUSE2:
        if (now - t0 >= 500)
            enterPhase(CW_RUN);
        break;
    }
#endif
}
