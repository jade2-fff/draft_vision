// 相机诊断工具：查询这台相机的实际参数范围、像素格式，并尝试取一帧
#include <cstdio>
#include <cstring>

#include "MvCameraControl.h"

static void show_float(void *h, const char *key) {
    MVCC_FLOATVALUE v{};
    int ret = MV_CC_GetFloatValue(h, key, &v);
    if (ret == MV_OK)
        printf("  %-14s cur=%.3f  min=%.3f  max=%.3f\n", key, v.fCurValue, v.fMin, v.fMax);
    else
        printf("  %-14s GetFloat failed: 0x%x\n", key, ret);
}

static void show_enum(void *h, const char *key) {
    MVCC_ENUMVALUE v{};
    int ret = MV_CC_GetEnumValue(h, key, &v);
    if (ret == MV_OK) {
        printf("  %-14s cur=%u  supported=[", key, v.nCurValue);
        for (unsigned i = 0; i < v.nSupportedNum; ++i)
            printf("%u ", v.nSupportValue[i]);
        printf("]\n");
    } else {
        printf("  %-14s GetEnum failed: 0x%x\n", key, ret);
    }
}

int main() {
    MV_CC_DEVICE_INFO_LIST list{};
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &list);
    if (ret != MV_OK) { printf("EnumDevices failed: 0x%x\n", ret); return 1; }
    if (list.nDeviceNum == 0) { printf("no camera found\n"); return 1; }

    printf("found %u device(s)\n", list.nDeviceNum);

    void *h = nullptr;
    ret = MV_CC_CreateHandle(&h, list.pDeviceInfo[0]);
    if (ret != MV_OK) { printf("CreateHandle failed: 0x%x\n", ret); return 1; }

    ret = MV_CC_OpenDevice(h);
    if (ret != MV_OK) { printf("OpenDevice failed: 0x%x\n", ret); return 1; }

    printf("\n=== current settings & ranges ===\n");
    show_float(h, "ExposureTime");
    show_float(h, "Gain");
    show_float(h, "AcquisitionFrameRate");
    show_enum(h, "PixelFormat");
    show_enum(h, "TriggerMode");
    show_enum(h, "ExposureAuto");
    show_enum(h, "GainAuto");
    show_enum(h, "AcquisitionMode");

    printf("\n=== configure like main program ===\n");
    ret = MV_CC_SetEnumValue(h, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    printf("  set ExposureAuto OFF: 0x%x\n", ret);
    ret = MV_CC_SetEnumValue(h, "GainAuto", MV_GAIN_MODE_OFF);
    printf("  set GainAuto OFF: 0x%x\n", ret);
    ret = MV_CC_SetEnumValue(h, "TriggerMode", MV_TRIGGER_MODE_OFF);
    printf("  set TriggerMode OFF: 0x%x\n", ret);
    ret = MV_CC_SetFloatValue(h, "ExposureTime", 4500.0f);
    printf("  set ExposureTime 4500: 0x%x\n", ret);
    ret = MV_CC_SetFloatValue(h, "Gain", 16.0f);
    printf("  set Gain 16: 0x%x\n", ret);

    printf("\n=== try grab one frame ===\n");
    ret = MV_CC_StartGrabbing(h);
    printf("  StartGrabbing: 0x%x\n", ret);

    MV_FRAME_OUT frame{};
    for (int i = 0; i < 5; ++i) {
        ret = MV_CC_GetImageBuffer(h, &frame, 1000);
        printf("  GetImageBuffer[%d]: 0x%x", i, ret);
        if (ret == MV_OK) {
            printf("  W=%d H=%d pixel=0x%x len=%d\n",
                   frame.stFrameInfo.nWidth, frame.stFrameInfo.nHeight,
                   frame.stFrameInfo.enPixelType, frame.stFrameInfo.nFrameLen);
            MV_CC_FreeImageBuffer(h, &frame);
            break;
        }
        printf("\n");
    }

    MV_CC_StopGrabbing(h);
    MV_CC_CloseDevice(h);
    MV_CC_DestroyHandle(h);
    return 0;
}
