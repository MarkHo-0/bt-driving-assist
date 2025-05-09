#ifndef CAR_MAP_H_
#define CAR_MAP_H_

#include <stdint.h>
#include "car_vector.h"

// --- 常數 ---
#define MAP_CHUNK_DIM 55   // 地圖區塊的維度，必須是單數
#define CHUNK_PIXEL_DIM 8

// --- 公用函數 ---
//所有函數都會檢查是否需要更新物理坐標系，並在需要時進行shift地圖和更新物理坐標系
//就算get類型函數也會進行這個檢查，這樣可以確保所有函數都能正確地獲取到最新的數據

Vec2i8 getMapCenterChunkCoord();

uint64_t getMapChunk(Vec2i8 chunk_coord);

void setMapChunk(Vec2i8 chunkCoord, uint64_t chunkData);

uint8_t getMapPixel(Vec2i16 pixelCoord);

void setMapPixel(Vec2i16 pixel_coord, uint8_t value);

void resetMap();

// 請在main.c中定義這個函數，不然會報錯
void onChunkUpdated(Vec2i8 chunk_coord, uint64_t chunkData);

#endif /* CAR_MAP_H_ */