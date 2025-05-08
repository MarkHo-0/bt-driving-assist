#include "car_map.h"

static const uint8_t PHYS_HALF_IDX = MAP_CHUNK_DIM / 2;

static uint64_t map[MAP_CHUNK_DIM][MAP_CHUNK_DIM] = {};
static Vec2i8 logicalCenterCoord = {0, 0};

static Vec2i8 lastWrittenChunkCoord = {0, 0};
static uint64_t lastChunkData = 0;

// 除法運算，確保結果向下取整
static inline int16_t floorDiv(int16_t a, int16_t b) {
    int16_t res = a / b;
    int16_t rem = a % b;
    // 如果有餘數，並且兩個數字的符號不同，則需要向下取整
    if ((rem != 0) && ((a < 0) != (b < 0))) {
        res--;
    }
    return res;
}

// 取模運算，確保結果為正數
static inline int16_t posMod(int16_t a, int16_t b) {
    int16_t res = a % b;
    return (res < 0) ? (res + b) : res;
}

static void performShift(Vec2i8 offset) {
    if (offset.x == 0 && offset.y == 0) return;
    
    int8_t dx = offset.x, dy = offset.y;
    uint8_t absDx = (dx > 0) ? dx : -dx, absDy = (dy > 0) ? dy : -dy;

    if (absDx > MAP_CHUNK_DIM || absDy > MAP_CHUNK_DIM) {
        memset(map, 0, sizeof(map));
        return;
    }

   // --- 垂直移動 (Y軸) ---
    if (dy != 0) {
        if (dy > 0) { // 向上移動 (記憶體中 index 增加)，清空底部 absDy 行
            memmove(&map[absDy][0], &map[0][0], (MAP_CHUNK_DIM - absDy) * MAP_CHUNK_DIM * sizeof(uint64_t));
            memset(&map[0][0], 0, absDy * MAP_CHUNK_DIM * sizeof(uint64_t));
        } else { // 向下移動 (記憶體中 index 減少)，清空頂部 absDy 行
            memmove(&map[0][0], &map[absDy][0], (MAP_CHUNK_DIM - absDy) * MAP_CHUNK_DIM * sizeof(uint64_t));
            memset(&map[MAP_CHUNK_DIM - absDy][0], 0, absDy * MAP_CHUNK_DIM * sizeof(uint64_t));
        }
    }

    // --- 水平移動 (X軸) ---
    if (dx != 0) {
        for (uint8_t r = 0; r < MAP_CHUNK_DIM; ++r) {
            // 跳過喺垂直移動中已經被清空嘅行 (避免喺 0 上面做 memmove)
            if (dy > 0 && r < absDy) continue;
            if (dy < 0 && r >= (MAP_CHUNK_DIM - absDy)) continue;

            if (dx > 0) { // 向右移動 (記憶體中 index 增加)，清空每行左邊 absDx 列
                memmove(&map[r][absDx], &map[r][0], (MAP_CHUNK_DIM - absDx) * sizeof(uint64_t));
                memset(&map[r][0], 0, absDx * sizeof(uint64_t)); 
            } else { // 向左移動 (記憶體中 index 減少)，清空每行右邊 absDx 列
                memmove(&map[r][0], &map[r][absDx], (MAP_CHUNK_DIM - absDx) * sizeof(uint64_t));
                memset(&map[r][MAP_CHUNK_DIM - absDx], 0, absDx * sizeof(uint64_t));
            }
        }
    }
}

static void ensureChunkMapped(Vec2i8 targetChunkCoord, Vec2ui8* phyChunkCoord) {
    // 計算目標區塊座標與邏輯中心座標的差值
    Vec2i8 diff = vec2i8_sub(targetChunkCoord, logicalCenterCoord);
    
    // 計算初始物理座標 (相對於地圖中心)
    Vec2i16 possiblePhyCoord = {diff.x + PHYS_HALF_IDX, diff.y + PHYS_HALF_IDX};

    // 計算需要移動地圖的偏移量
    Vec2i8 shift = {0, 0};
    
    // X軸方向檢查
    if (possiblePhyCoord.x < 0) {
        shift.x = possiblePhyCoord.x;  // 需要向右移動地圖(正X方向)
    } else if (possiblePhyCoord.x >= MAP_CHUNK_DIM) {
        shift.x = possiblePhyCoord.x - MAP_CHUNK_DIM + 1;  // 需要向左移動地圖(負X方向)
    }
    
    // Y軸方向檢查
    if (possiblePhyCoord.y < 0) {
        shift.y = possiblePhyCoord.y;  // 需要向下移動地圖(記憶體中的負Y方向)
    } else if (possiblePhyCoord.y >= MAP_CHUNK_DIM) {
        shift.y = possiblePhyCoord.y - MAP_CHUNK_DIM + 1;  // 需要向上移動地圖(記憶體中的正Y方向)
    }

    // 如果需要移動地圖
    if (shift.x != 0 || shift.y != 0) {
        performShift(shift);  // 執行地圖移動

        // 更新邏輯中心坐標
        logicalCenterCoord = vec2i8_add(logicalCenterCoord, shift);

        // 重新計算物理座標 (shift 之後，目標區塊應該已經喺 map 邊界內)
        diff = vec2i8_sub(targetChunkCoord, logicalCenterCoord);
        possiblePhyCoord.x = (int16_t)diff.x + PHYS_HALF_IDX;
        possiblePhyCoord.y = (int16_t)diff.y + PHYS_HALF_IDX;
    }

    // 最終物理座標 (此時保證在地圖邊界內)
    *phyChunkCoord = (Vec2ui8){
        .x = (uint8_t)possiblePhyCoord.x,
        .y = (uint8_t)possiblePhyCoord.y
    };
}

// 將全局像素座標轉換為 (區塊座標 + 區塊內局部座標)
static inline void pixelCoordFromGlobalToLocal(Vec2i16 pixelCoord, Vec2i8* chunkCoord, Vec2i8* localCoord) {
    // X-direction (standard)
    chunkCoord->x = (int8_t)floorDiv(pixelCoord.x, CHUNK_PIXEL_DIM);
    localCoord->x = posMod(pixelCoord.x, CHUNK_PIXEL_DIM);

    // Y-direction (adjusted)
    if (pixelCoord.y >= 0) {
        chunkCoord->y = (int8_t)floorDiv(pixelCoord.y, CHUNK_PIXEL_DIM);
        localCoord->y = posMod(pixelCoord.y, CHUNK_PIXEL_DIM);
        if (pixelCoord.y > 0) {
            chunkCoord->y += 1;
            localCoord->y = 7 - posMod(pixelCoord.y - 1, CHUNK_PIXEL_DIM);
        }
    } else { // y < 0
        chunkCoord->y = (int8_t)floorDiv(pixelCoord.y, CHUNK_PIXEL_DIM);
        localCoord->y = posMod(pixelCoord.y, CHUNK_PIXEL_DIM);
    }
}

Vec2i8 getMapCenterChunkCoord() {
    return logicalCenterCoord;
}

uint64_t getMapChunk(Vec2i8 chunkCoord) {
    Vec2ui8 phyChunkCoord = {0, 0};
    ensureChunkMapped(chunkCoord, &phyChunkCoord);
    return map[phyChunkCoord.y][phyChunkCoord.x];
}

void setMapChunk(Vec2i8 chunkCoord, uint64_t chunkData) {
    // 如果數據相同，則不需要更新
    uint64_t oldChunkData = getMapChunk(chunkCoord);
    if (oldChunkData == chunkData) return;

    Vec2ui8 phyChunkCoord = {0, 0};
    ensureChunkMapped(chunkCoord, &phyChunkCoord);

    // 更新區塊數據
    map[phyChunkCoord.y][phyChunkCoord.x] = chunkData;

    // 呼叫回調函數
    onChunkUpdated(chunkCoord, chunkData); 

    // 更新最後寫入的區塊
    lastWrittenChunkCoord = chunkCoord; 
    lastChunkData = chunkData;
}

uint8_t getMapPixel(Vec2i16 pixelCoord) {
    Vec2i8 chunkCoord = {0, 0};  
    Vec2i8 phyLocalCoord = {0, 0};
    pixelCoordFromGlobalToLocal(pixelCoord, &chunkCoord, &phyLocalCoord);

    Vec2ui8 phyChunkCoord = {0, 0};
    ensureChunkMapped(chunkCoord, &phyChunkCoord);
    
    // 計算位元索引，然後從區塊數據中獲取對應的位元值
    uint64_t bitIndex = (uint64_t)phyLocalCoord.y * CHUNK_PIXEL_DIM + (uint64_t)phyLocalCoord.x;
    uint64_t chunkData = map[phyChunkCoord.y][phyChunkCoord.x];
    return (chunkData >> bitIndex) & 1ULL;
}

void setMapPixel(Vec2i16 pixelCoord, uint8_t value) {
    Vec2i8 chunkCoord = {0, 0};  
    Vec2i8 phyLocalCoord = {0, 0};
    pixelCoordFromGlobalToLocal(pixelCoord, &chunkCoord, &phyLocalCoord);

    Vec2ui8 phyChunkCoord = {0, 0};
    ensureChunkMapped(chunkCoord, &phyChunkCoord);

    // 如果 pixel 數據相同，則不需要更新
    uint64_t oldChunkData = map[phyChunkCoord.y][phyChunkCoord.x];
    uint64_t bitIndex = (uint64_t)phyLocalCoord.y * CHUNK_PIXEL_DIM + (uint64_t)phyLocalCoord.x;
    uint8_t oldPixelValue = (oldChunkData >> bitIndex) & 1ULL;
    if (oldPixelValue == value) return;

    // 更新 chunk 數據，因為前面確定數據不相同，所以可以直接 XOR
    oldChunkData ^= (1ULL << bitIndex);
    map[phyChunkCoord.y][phyChunkCoord.x] = oldChunkData;

    // 如果是最後寫入的區塊，則不需回調函數，只需要更緩存的數據
    if (vec2i8_equal(chunkCoord, lastWrittenChunkCoord) == 1)  {
        lastChunkData = oldChunkData;
        return;
    }

    // 當前 pixel 數據與最後寫入的區塊數據不同，代表完成了對最後寫入區塊的更新，需要回調通知
    onChunkUpdated(lastWrittenChunkCoord, lastChunkData);
    lastWrittenChunkCoord = chunkCoord;
    lastChunkData = oldChunkData;
}

//由 viewerCoord 到 obstacleCoord 畫條直線，清空條路線上面嘅 pixel，最後將 obstacleCoord 標記返做 1
void setObstaclePixel(Vec2i16 viewerCoord, Vec2i16 obstacleCoord, uint8_t maxDistance) {
    int16_t x0 = viewerCoord.x, y0 = viewerCoord.y;
    int16_t x1 = obstacleCoord.x, y1 = obstacleCoord.y;

    // 計算距離（用平方避免浮點數）
    int16_t dxDist = x1 - x0;
    int16_t dyDist = y1 - y0;
    int32_t distSquared = dxDist * dxDist + dyDist * dyDist;
    int32_t maxDistSquared = maxDistance * maxDistance;

    // 如果 obstacle 超出 maxDistance，就根據 maxDistance 計算新的 x1, y1
    if (distSquared > maxDistSquared) {
        float dist = sqrtf((float)distSquared);
        float ratio = maxDistance / dist;
        x1 = x0 + (int16_t)((x1 - x0) * ratio);
        y1 = y0 + (int16_t)((y1 - y0) * ratio);
    }

    int16_t dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy, e2;
    Vec2i16 currentPixel;

    while (1) {
        currentPixel.x = x0;
        currentPixel.y = y0;
        setMapPixel(currentPixel, 0); // 清空當前點

        if (x0 == x1 && y0 == y1) break; // 到達終點就停

        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }

    // 只有在原始距離小於等於 maxDistance 時，才設置障礙物
    if (distSquared <= maxDistSquared) {
        setMapPixel(obstacleCoord, 1);
    }
}

void resetMap() {
    memset(map, 0, sizeof(map));
    logicalCenterCoord = (Vec2i8){0, 0};
    lastWrittenChunkCoord = (Vec2i8){0, 0};
}

void generateChunkPackage(Vec2i8 chunkCoord, uint64_t chunkData, uint8_t* package) {
    package[0] = 0;
    package[1] = (uint8_t)chunkCoord.x; // X坐標
    package[2] = (uint8_t)chunkCoord.y; // Y坐標

    for (uint8_t i = 0; i < 8; i++) {
        package[i + 3] = (chunkData >> ((7 - i) * 8)) & 0xFF; 
    }
}


