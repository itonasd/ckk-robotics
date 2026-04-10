#include <PTBOTAtomVX.h>
#include <Bluepad32.h>

#define MAX_CONTROLLERS BP32_MAX_GAMEPADS
ControllerPtr myControllers[MAX_CONTROLLERS];

typedef struct _ControllerData {
    int8_t index, l1, r1, up, down, left, right, triangle, cross, square, circle; // 0, 1 boolean
    double lx, ly, rx, ry, l2, r2, battery; // 0.00 - 1.00 range
} ControllerData_t;

ControllerData_t controller;

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < MAX_CONTROLLERS; i++) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            return;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < MAX_CONTROLLERS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            return;
        }
    }
}

uint8_t getControllerData(ControllerData_t* ControllerData) {
    if (!BP32.update()) return 0;
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
            uint8_t buffdpad = ctl->dpad();
            uint8_t buffbutton = ctl->buttons();

            ControllerData->index = ctl->index();
            ControllerData->lx = (double) ctl->axisX() / 512.0;
            ControllerData->ly = (double) ctl->axisY() / 512.0;
            ControllerData->rx = (double) ctl->axisRX() / 512.0;
            ControllerData->ry = (double) ctl->axisRY() / 512.0;
            ControllerData->l1 = ctl->l1();
            ControllerData->r1 = ctl->r1();
            ControllerData->l2 = (double) ctl->brake() / 1020.0;
            ControllerData->r2 = (double) ctl->throttle() / 1020.0;
            ControllerData->up = (buffdpad >> 0) & 1;
            ControllerData->down = (buffdpad >> 1) & 1;
            ControllerData->left = (buffdpad >> 3) & 1;
            ControllerData->right = (buffdpad >> 2) & 1;
            ControllerData->triangle = (buffbutton >> 3) & 1;
            ControllerData->cross = (buffbutton >> 0) & 1;
            ControllerData->square = (buffbutton >> 2) & 1;
            ControllerData->circle = (buffbutton >> 1) & 1;
            ControllerData->battery = (double) ctl->battery() / 255.0;
        }
    }

    return 1;
}


float Power_FL, Power_FR, Power_BL, Power_BR;
float yawOffset = 0;
int spd = 40;

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    initialize();
    // calibrateIMU();

    // yawOffset = angleRead(YAW);

    BP32.setup(&onConnectedController, &onDisconnectedController);

}


float getYawRad() {
      float yaw = angleRead(YAW) - yawOffset;

      if (yaw > 180) yaw -= 360;
      if (yaw < -180) yaw += 360;

      return yaw * PI / 180.0;

}


void loop() {
    getControllerData(&controller);

    float yawRad = getYawRad();
  
    float strafe = controller.lx;
    float forward = -controller.ly;
    float turn = controller.rx;

    float rotX = strafe * cos(yawRad) - forward * sin(yawRad);
    float rotY = strafe * sin(yawRad) + forward * cos(yawRad);

    Power_FL = rotY + rotX + turn;
    Power_FR = rotY - rotX - turn;
    Power_BL = rotY - rotX + turn;
    Power_BR = rotY + rotX - turn;

    float maxVal = max(
      max(abs(Power_FL), abs(Power_FR)),
      max(abs(Power_BL), abs(Power_BR))
    );

    motorWrite(1,Power_FL * spd);
    motorWrite(2,Power_FR * spd);
    motorWrite(3,Power_BL * spd);
    motorWrite(4,Power_BR * spd);

    if (controller.circle) {
      yawOffset = angleRead(YAW);
    }

    Serial.print(yawRad);
    Serial.print("\n");

    delay(10);
}
