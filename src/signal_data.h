const uint8_t teslaSequence[43]  = { 
    0x02,0xAA,0xAA,0xAA,  // Preamble of 26 bits by repeating 1010
    0x2B,                 // Sync byte
    0x2C,0xCB,0x33,0x33,0x2D,0x34,0xB5,0x2B,0x4D,0x32,0xAD,0x2C,0x56,0x59,0x96,0x66,
    0x66,0x5A,0x69,0x6A,0x56,0x9A,0x65,0x5A,0x58,0xAC,0xB3,0x2C,0xCC,0xCC,0xB4,0xD2,
    0xD4,0xAD,0x34,0xCA,0xB4,0xA0};

const uint8_t bluetooth_channels[] = {32, 34, 46, 48, 50, 52, 0, 1, 2, 4, 6, 8, 22, 24, 26, 28, 30, 74, 76, 78, 80};
const uint8_t ble_channels[] = {2, 26, 80};
const uint8_t wifi_channels[] = {
        6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18,         // Channel 1
        22, 24, 26, 28,                                             // mid band
        30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, // Channel 6
        46, 48, 50, 52,                                             // mid band
        55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68,     // Channel 11
    }; // WiFi Channels
const uint8_t usb_channels[] = {40, 50, 60};   // USB Wireless Channels
const uint8_t video_channels[] = {70, 75, 80}; // Video Streaming Channels
const uint8_t rc_channels[] = {1, 3, 5, 7};    // RC Toys/ Drones Channels
const uint8_t full_channels[] = {
        1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
        22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
        43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
        64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84,
        85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
        101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112,
        113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124
    }; // All Channels
const uint8_t noiseAddress[][2] = {{0x55, 0x55}, {0xAA, 0xAA}, {0xA0, 0xAA}, {0xAB, 0xAA}, {0xAC, 0xAA}, {0xAD, 0xAA}};

const uint8_t bManCode[6] = {0x48,0xa9,0xca,0xe6,0xd8,0xc2};
