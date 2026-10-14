#include "config.h"

static inline int opposed(int dir) {
    return (dir + 2) & 3;
}

static void wall_sample() {
    int v[NUM_SENSORS] = {};
    const int NS = 5;
    for (int r = 0; r < NS; r++) {
        ir_sample();
        for (int i = 0; i < NUM_SENSORS; i++)
            v[i] += w[i];
        delay(3);
    }
    for (int i = 0; i < NUM_SENSORS; i++)
        w[i] = (v[i] >= 3) ? 1 : 0;
}

static inline int quality_get(uint8_t cf, int dir) {
    return (cf >> (dir * 2)) & 3;
}

static inline uint8_t quality_put(uint8_t cf, int dir, int v) {
    return (uint8_t)((cf & ~(3 << (dir * 2))) | ((v & 3) << (dir * 2)));
}

static void merge_edge(int x, int y, int dir, bool present, bool certain) {
    bool known = (seen[y][x] & edgeMask[dir]) != 0;
    bool cur = (edges[y][x] & edgeMask[dir]) != 0;
    if (!known || certain) {
        seen[y][x] |= edgeMask[dir];
        if (present)
            edges[y][x] |= edgeMask[dir];
        else
            edges[y][x] &= ~edgeMask[dir];
        quality[y][x] = quality_put(quality[y][x], dir, certain ? 3 : 2);
        return;
    }
    int cf = quality_get(quality[y][x], dir);
    if (present == cur) {
        if (cf < 3)
            quality[y][x] = quality_put(quality[y][x], dir, cf + 1);
    } else if (cf > 1) {
        quality[y][x] = quality_put(quality[y][x], dir, cf - 1);
    } else {
        if (present)
            edges[y][x] |= edgeMask[dir];
        else
            edges[y][x] &= ~edgeMask[dir];
        quality[y][x] = quality_put(quality[y][x], dir, 1);
    }
}

static void put_edge(int x, int y, int dir, bool present, bool certain = false) {
    if (x < 0 || x >= dim || y < 0 || y >= dim)
        return;
    int nx = x + dx[dir], ny = y + dy[dir];
    if (nx < 0 || nx >= dim || ny < 0 || ny >= dim) {
        seen[y][x] |= edgeMask[dir];
        edges[y][x] |= edgeMask[dir];
        quality[y][x] = quality_put(quality[y][x], dir, 3);
        return;
    }
    merge_edge(x, y, dir, present, certain);
    merge_edge(nx, ny, opposed(dir), present, certain);
}

void map_init() {
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 16; x++) {
            edges[y][x] = 0;
            field[y][x] = 255;
            seen[y][x] = 0;
            quality[y][x] = 0;
            visited[y][x] = false;
        }
    routeLen = 0;
    for (int i = 0; i < dim; i++) {
        edges[0][i] |= WALL_S;
        edges[dim - 1][i] |= WALL_N;
        edges[i][0] |= WALL_W;
        edges[i][dim - 1] |= WALL_E;
    }
}

void target_box(int &x0, int &y0, int &x1, int &y1) {
    if (targetMode == 0) {
        x0 = y0 = (dim - 1) / 2;
        x1 = (dim % 2 == 1) ? x0 : x0 + 1;
        y1 = (dim % 2 == 1) ? y0 : y0 + 1;
    } else {
        x0 = constrain(tx0, 0, dim - 1);
        y0 = constrain(ty0, 0, dim - 1);
        x1 = (targetMode == 2) ? min(x0 + 1, dim - 1) : x0;
        y1 = (targetMode == 2) ? min(y0 + 1, dim - 1) : y0;
    }
    if (x1 >= dim)
        x1 = dim - 1;
    if (y1 >= dim)
        y1 = dim - 1;
}

void propagate() {
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++)
            field[y][x] = 255;
    static uint8_t qx[300], qy[300];
    int head = 0, tail = 0;
    int gx, gy, gx2, gy2;
    target_box(gx, gy, gx2, gy2);
    for (int cy = gy; cy <= gy2 && cy < dim; cy++) {
        for (int cx = gx; cx <= gx2 && cx < dim; cx++) {
            field[cy][cx] = 0;
            qx[tail] = cx;
            qy[tail] = cy;
            tail++;
        }
    }
    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        uint8_t nd = field[cy][cx] + 1;
        for (int dir = 0; dir < 4; dir++) {
            if (edges[cy][cx] & edgeMask[dir])
                continue;
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];
            if (nx < 0 || nx >= dim || ny < 0 || ny >= dim)
                continue;
            if (field[ny][nx] > nd && tail < 300) {
                field[ny][nx] = nd;
                qx[tail] = nx;
                qy[tail] = ny;
                tail++;
            }
        }
    }
}

void propagate_to(int gx, int gy) {
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++)
            field[y][x] = 255;
    static uint8_t qx[300], qy[300];
    int head = 0, tail = 0;
    if (gx >= 0 && gx < dim && gy >= 0 && gy < dim) {
        field[gy][gx] = 0;
        qx[tail] = (uint8_t)gx;
        qy[tail] = (uint8_t)gy;
        tail++;
    }
    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;
        uint8_t nd = field[cy][cx] + 1;
        for (int dir = 0; dir < 4; dir++) {
            if (edges[cy][cx] & edgeMask[dir])
                continue;
            int nx = cx + dx[dir], ny = cy + dy[dir];
            if (nx < 0 || nx >= dim || ny < 0 || ny >= dim)
                continue;
            if (field[ny][nx] > nd && tail < 300) {
                field[ny][nx] = nd;
                qx[tail] = (uint8_t)nx;
                qy[tail] = (uint8_t)ny;
                tail++;
            }
        }
    }
}

void map_sample(int x, int y) {
    static const int SI[3] = {SENS_L, SENS_F, SENS_R};
    int dirs[3];
    sensor_dirs(dirs);
    for (int slot = 0; slot < 3; slot++)
        put_edge(x, y, dirs[slot], w[SI[slot]] != 0);
}

int choose_dir() {
    uint8_t cur = field[cy0][cx0];
    uint8_t best = cur;
    int bestDir = -1;
    int bestCost = 99;
    for (int dir = 0; dir < 4; dir++) {
        if (edges[cy0][cx0] & edgeMask[dir])
            continue;
        int nx = cx0 + dx[dir], ny = cy0 + dy[dir];
        if (nx < 0 || nx >= dim || ny < 0 || ny >= dim)
            continue;
        uint8_t f = field[ny][nx];
        if (f >= cur)
            continue;
        int rel = (dir - dir0 + 4) & 3;
        int cost = (rel == 0) ? 0 : (rel == 2) ? 2 : 1;
        if (f < best || (f == best && cost < bestCost)) {
            best = f;
            bestDir = dir;
            bestCost = cost;
        }
    }
    return bestDir;
}

void face(int newDir) {
    turnFault = false;
    if (newDir == dir0)
        return;
    int delta = (newDir - dir0 + 4) & 3;
    rotate(delta == 1 ? 1 : delta == 3 ? 0 : 2);
    if (!turnFault)
        dir0 = newDir;
}

void advance() {
    int fromX = cx0, fromY = cy0, dir = dir0;
    drive_mm(pitch);
    if (!blocked) {
        put_edge(fromX, fromY, dir, false, true);
        cx0 += dx[dir];
        cy0 += dy[dir];
        if (cx0 >= 0 && cx0 < dim && cy0 >= 0 && cy0 < dim)
            visited[cy0][cx0] = true;
    }
}

void trace(int sx, int sy) {
    routeLen = 0;
    int x = sx, y = sy;
    while (field[y][x] != 0 && routeLen < SOLVE_PATH_MAX) {
        uint8_t best = 255;
        int bestDir = -1;
        for (int dir = 0; dir < 4; dir++) {
            if (edges[y][x] & edgeMask[dir])
                continue;
            int nx = x + dx[dir], ny = y + dy[dir];
            if (nx < 0 || nx >= dim || ny < 0 || ny >= dim)
                continue;
            if (field[ny][nx] < best) {
                best = field[ny][nx];
                bestDir = dir;
            }
        }
        if (bestDir == -1)
            break;
        route[routeLen++] = (uint8_t)bestDir;
        x += dx[bestDir];
        y += dy[bestDir];
    }
}

bool route_valid() {
    if (routeLen == 0)
        return false;
    int sX = origin_cell();
    int x = sX, y = 0;
    for (int i = 0; i < routeLen; i++) {
        int dir = route[i];
        if (dir < 0 || dir > 3)
            return false;
        if (!(seen[y][x] & edgeMask[dir]) || (edges[y][x] & edgeMask[dir]))
            return false;
        int nx = x + dx[dir], ny = y + dy[dir];
        if (nx < 0 || nx >= dim || ny < 0 || ny >= dim)
            return false;
        if (!(seen[ny][nx] & edgeMask[opposed(dir)]) || (edges[ny][nx] & edgeMask[opposed(dir)]))
            return false;
        x = nx;
        y = ny;
    }
    int gx0, gy0, gx1, gy1;
    target_box(gx0, gy0, gx1, gy1);
    return x >= gx0 && x <= gx1 && y >= gy0 && y <= gy1;
}

static uint8_t map_sum() {
    uint8_t sum = MAZE_MAGIC ^ (uint8_t)dim;
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++)
            sum = (uint8_t)(sum * 31 + edges[y][x] + seen[y][x]);
    sum = (uint8_t)(sum * 31 + (uint8_t)routeLen);
    for (int i = 0; i < routeLen; i++)
        sum = (uint8_t)(sum * 31 + route[i]);
    return sum;
}

void map_save() {
    int pathBytes = (routeLen < SOLVE_PATH_MAX) ? routeLen : SOLVE_PATH_MAX;
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++) {
            EEPROM.write(MAZE_WALLS_EEPROM + y * 16 + x, edges[y][x]);
            EEPROM.write(MAZE_KNOWN_EEPROM + y * 16 + x, seen[y][x]);
        }
    for (int i = 0; i < pathBytes; i++)
        EEPROM.write(PATH_EEPROM_BASE + i, route[i]);
    EEPROM.write(MAZE_SIZE_ADDR, (uint8_t)dim);
    EEPROM.write(PATH_LEN_ADDR, (uint8_t)(pathBytes >> 8));
    EEPROM.write(PATH_LEN_ADDR + 1, (uint8_t)(pathBytes & 0xFF));
    EEPROM.write(MAZE_CRC_ADDR, map_sum());
    EEPROM.write(MAZE_MAGIC_ADDR, MAZE_MAGIC);
    delay(10);
    EEPROM.commit();
}

bool map_load() {
    if (EEPROM.read(MAZE_MAGIC_ADDR) != MAZE_MAGIC)
        return false;
    if (EEPROM.read(MAZE_SIZE_ADDR) != (uint8_t)dim)
        return false;
    static uint8_t savedWalls[16][16], savedKnown[16][16];
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++) {
            savedWalls[y][x] = EEPROM.read(MAZE_WALLS_EEPROM + y * 16 + x);
            savedKnown[y][x] = EEPROM.read(MAZE_KNOWN_EEPROM + y * 16 + x);
        }
    int len = ((int)EEPROM.read(PATH_LEN_ADDR) << 8) | EEPROM.read(PATH_LEN_ADDR + 1);
    if (len < 0 || len > SOLVE_PATH_MAX)
        return false;
    static uint8_t savedPath[SOLVE_PATH_MAX];
    for (int i = 0; i < len; i++) {
        savedPath[i] = EEPROM.read(PATH_EEPROM_BASE + i);
        if (savedPath[i] > 3)
            return false;
    }
    uint8_t sum = MAZE_MAGIC ^ (uint8_t)dim;
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++)
            sum = (uint8_t)(sum * 31 + savedWalls[y][x] + savedKnown[y][x]);
    sum = (uint8_t)(sum * 31 + (uint8_t)len);
    for (int i = 0; i < len; i++)
        sum = (uint8_t)(sum * 31 + savedPath[i]);
    if (sum != EEPROM.read(MAZE_CRC_ADDR))
        return false;
    for (int y = 0; y < dim; y++)
        for (int x = 0; x < dim; x++) {
            edges[y][x] = savedWalls[y][x];
            seen[y][x] = savedKnown[y][x];
            quality[y][x] = savedKnown[y][x] ? 0xFF : 0;
        }
    routeLen = len;
    for (int i = 0; i < len; i++)
        route[i] = savedPath[i];
    return true;
}

void map_clear() {
    for (int i = MAZE_MAGIC_ADDR; i <= MAZE_CRC_ADDR; i++)
        EEPROM.write(i, 0);
    for (int i = MAZE_KNOWN_EEPROM; i < MAZE_KNOWN_EEPROM + 256; i++)
        EEPROM.write(i, 0);
    delay(10);
    EEPROM.commit();
    map_init();
    routeLen = 0;
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(1);
    oled.setCursor(8, 22);
    oled.print("MAZE CLR");
    oled.display();
    delay(1000);
}

enum NavState { NAV_NEXT, NAV_DONE, NAV_STOP, NAV_BLOCKED };

static void field_update(bool single, int gx, int gy) {
    if (single)
        propagate_to(gx, gy);
    else
        propagate();
}

static NavState search_step(bool single, int gx, int gy, int &stuckCount, int &turnsHere,
                            const char *tag) {
    sample_pose();
    wall_sample();
    uint8_t before = edges[cy0][cx0];
    map_sample(cx0, cy0);
    if (edges[cy0][cx0] != before)
        field_update(single, gx, gy);
    if (field[cy0][cx0] == 0)
        return NAV_DONE;
    oled.clearDisplay();
    put_text(tag, 0, 0, 1, 1);
    put_text(String(cx0) + "," + String(cy0) + " " + String("NESW"[dir0]), 54, 0, 1, 1);
    put_text("Fl:" + String(field[cy0][cx0]), 0, 10, 1, 1);
    for (byte i = 0; i < NUM_SENSORS; i++) {
        int h = map(s[i], 0, 4095, 0, 25);
        oled.fillRect(14 + 22 * i, 40 - h, 14, h, 1);
    }
    put_text("L/R=STOP", 22, 54, 1, 1);
    oled.display();
    if ((!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT))) {
        drive(0, 0);
        wait_motion(200);
        return NAV_STOP;
    }
    int nextDir = choose_dir();
    if (nextDir == -1) {
        if (++stuckCount > 5)
            return NAV_BLOCKED;
        face(opposed(dir0));
        if (turnFault)
            return NAV_STOP;
        field_update(single, gx, gy);
        return NAV_NEXT;
    }
    stuckCount = 0;
    int prevDir = dir0;
    face(nextDir);
    if (turnFault)
        return NAV_STOP;
    if (nextDir != prevDir)
        turnsHere++;
    wait_motion(POST_TURN_SETTLE_MS);
    wall_sample();
    before = edges[cy0][cx0];
    map_sample(cx0, cy0);
    if (edges[cy0][cx0] != before)
        field_update(single, gx, gy);
    if (w[SENS_F]) {
        if (turnsHere > 4)
            return NAV_BLOCKED;
        return NAV_NEXT;
    }
    int fromX = cx0, fromY = cy0;
    advance();
    if (blocked) {
        put_edge(fromX, fromY, dir0, true, true);
        field_update(single, gx, gy);
        blocked = false;
    }
    if (debug)
        debugf("[%s] %d,%d %c fl:%d turns:%d\n", tag, cx0, cy0, "NESW"[dir0], field[cy0][cx0],
               turnsHere);
    return NAV_NEXT;
}

static NavState search_to(bool single, int gx, int gy, const char *tag) {
    field_update(single, gx, gy);
    int stuckCount = 0, turnsHere = 0;
    while (1) {
        int px = cx0, py = cy0;
        NavState r = search_step(single, gx, gy, stuckCount, turnsHere, tag);
        if (r != NAV_NEXT)
            return r;
        if (cx0 != px || cy0 != py)
            turnsHere = 0;
    }
}

static bool replay(bool reverse, short int runSpeed) {
    if (routeLen == 0)
        return false;
    bool success = true;
    short int savedSpeed = pwm;
    short int savedBrake = brakeMs;
    pwm = runSpeed;
    baseL = runSpeed;
    baseR = runSpeed;
    brakeMs = (short int)constrain(
        (int)((float)savedBrake * runSpeed / (float)(savedSpeed > 0 ? savedSpeed : 100)), 0, 500);
    for (int step = 0; step < routeLen; step++) {
        int idx = reverse ? (routeLen - 1 - step) : step;
        int dir = reverse ? opposed((int)route[idx]) : (int)route[idx];
        face(dir);
        if (turnFault) {
            success = false;
            break;
        }
        ir_sample();
        drive_mm(pitch);
        if (blocked) {
            success = false;
            drive(0, 0);
            break;
        }
        cx0 += dx[dir0];
        cy0 += dy[dir0];
        if (debug)
            debugf("[RUN%s] %d/%d dir:%c pos:%d,%d\n", reverse ? "R" : "F", step + 1, routeLen,
                   "NESW"[dir0], cx0, cy0);
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            success = false;
            drive(0, 0);
            wait_motion(200);
            break;
        }
    }
    drive(0, 0);
    wait_motion(300);
    pwm = savedSpeed;
    baseL = savedSpeed;
    baseR = savedSpeed;
    brakeMs = savedBrake;
    return success;
}

static bool replay_blocks(bool reverse, short int runSpeed) {
    if (routeLen == 0)
        return false;
    bool success = true;
    short int savedSpeed = pwm;
    short int savedBrake = brakeMs;
    pwm = runSpeed;
    baseL = runSpeed;
    baseR = runSpeed;
    brakeMs = (short int)constrain(
        (int)((float)savedBrake * runSpeed / (float)(savedSpeed > 0 ? savedSpeed : 100)), 0, 500);
    int step = 0;
    while (step < routeLen) {
        int idx = reverse ? (routeLen - 1 - step) : step;
        int dir = reverse ? opposed((int)route[idx]) : (int)route[idx];
        int count = 1;
        while (step + count < routeLen) {
            int nIdx = reverse ? (routeLen - 1 - (step + count)) : (step + count);
            int nDir = reverse ? opposed((int)route[nIdx]) : (int)route[nIdx];
            if (nDir != dir)
                break;
            count++;
        }
        face(dir);
        if (turnFault) {
            success = false;
            break;
        }
        ir_sample();
        drive_mm((float)count * pitch);
        if (blocked) {
            success = false;
            drive(0, 0);
            break;
        }
        cx0 += dx[dir0] * count;
        cy0 += dy[dir0] * count;
        if (debug)
            debugf("[SEG%s] step:%d dir:%c cnt:%d pos:%d,%d\n", reverse ? "R" : "F", step,
                   "NESW"[dir0], count, cx0, cy0);
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            success = false;
            drive(0, 0);
            wait_motion(200);
            break;
        }
        step += count;
    }
    drive(0, 0);
    wait_motion(300);
    pwm = savedSpeed;
    baseL = savedSpeed;
    baseR = savedSpeed;
    brakeMs = savedBrake;
    return success;
}

static bool check_route(int sX) {
    bool ok = false;
    int budget = dim * dim / 2;
    int spent = 0;
    for (int attempt = 1; attempt <= 6 && spent < budget; attempt++) {
        propagate();
        trace(sX, 0);
        if (routeLen == 0)
            break;
        if (route_valid()) {
            ok = true;
            break;
        }
        int x = sX, y = 0, tx = -1, ty = -1;
        for (int i = 0; i < routeLen; i++) {
            x += dx[route[i]];
            y += dy[route[i]];
            if (x < 0 || x >= dim || y < 0 || y >= dim)
                break;
            if (!visited[y][x]) {
                tx = x;
                ty = y;
                break;
            }
        }
        if (tx < 0) {
            ok = true;
            break;
        }
        oled.clearDisplay();
        put_text("PHASE 2.5", 20, 4, 1, 1);
        put_text("VERIFY PATH", 4, 16, 1, 1);
        put_text("try " + String(attempt) + " -> " + String(tx) + "," + String(ty), 4, 30, 1, 1);
        put_text("L/R=STOP", 22, 54, 1, 1);
        oled.display();
        delay(600);
        int bx = cx0, by = cy0;
        if (search_to(true, tx, ty, "VERIFY") != NAV_DONE)
            break;
        spent += abs(cx0 - bx) + abs(cy0 - by);
    }
    if (cx0 != sX || cy0 != 0)
        if (search_to(true, sX, 0, "HOME") != NAV_DONE)
            return false;
    return ok;
}

void follow_hand(int rule) {
    map_init();
    int sX = origin_cell();
    int gx0, gy0, gx1, gy1;
    target_box(gx0, gy0, gx1, gy1);
    cx0 = sX;
    cy0 = 0;
    dir0 = DIR_N;
    pose_zero();
    visited[0][sX] = true;
    const char *tag = (rule == 0) ? "LEFT HAND" : "RIGHT HAND";
    oled.clearDisplay();
    put_text(tag, 24, 4, 1, 1);
    put_text("WALL FOLLOW", 14, 18, 1, 1);
    put_text("Goal " + String(gx0) + "," + String(gy0) + " Sz" + String(dim), 0, 30, 1, 1);
    put_text("L/R=STOP", 22, 54, 1, 1);
    oled.display();
    {
        sound_start();
        delay(1500);
    }
    drive_mm(CELL_ENTRY_MM);
    const int STEP_LIMIT = 8 * dim * dim;
    static const int SI[3] = {SENS_L, SENS_F, SENS_R};
    int stalls = 0;
    for (int step = 0; step < STEP_LIMIT; step++) {
        sample_pose();
        wall_sample();
        map_sample(cx0, cy0);
        if (cx0 >= gx0 && cx0 <= gx1 && cy0 >= gy0 && cy0 <= gy1) {
            drive(0, 0);
            sound_goal();
            oled.clearDisplay();
            put_text("GOAL!", 22, 6, 1, 2);
            put_text("Pos:" + String(cx0) + "," + String(cy0), 10, 28, 1, 1);
            put_text(tag, 10, 40, 1, 1);
            oled.display();
            delay(2000);
            return;
        }
        oled.clearDisplay();
        put_text(tag, 0, 0, 1, 1);
        put_text(String(cx0) + "," + String(cy0) + " " + String("NESW"[dir0]), 84, 0, 1, 1);
        for (byte i = 0; i < NUM_SENSORS; i++) {
            int h = map(s[i], 0, 4095, 0, 25);
            oled.fillRect(14 + 22 * i, 40 - h, 14, h, 1);
        }
        put_text("L/R=STOP", 22, 54, 1, 1);
        oled.display();
        if ((!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT))) {
            drive(0, 0);
            wait_motion(200);
            return;
        }
        int dirs[3];
        sensor_dirs(dirs);
        int order[3];
        if (rule == 0) {
            order[0] = 0;
            order[1] = 1;
            order[2] = 2;
        } else {
            order[0] = 2;
            order[1] = 1;
            order[2] = 0;
        }
        int chosen = -1;
        for (int k = 0; k < 3 && chosen < 0; k++) {
            int slot = order[k];
            if (w[SI[slot]])
                continue;
            int d = dirs[slot];
            int nx = cx0 + dx[d], ny = cy0 + dy[d];
            if (nx < 0 || nx >= dim || ny < 0 || ny >= dim)
                continue;
            chosen = d;
        }
        if (chosen < 0)
            chosen = opposed(dir0);
        int prevDir = dir0;
        face(chosen);
        if (turnFault) {
            drive(0, 0);
            return;
        }
        advance();
        if (blocked) {
            put_edge(cx0, cy0, dir0, true, true);
            blocked = false;
            if (++stalls > 8)
                break;
            continue;
        }
        stalls = 0;
    }
    drive(0, 0);
    sound_error();
    oled.clearDisplay();
    put_text("NO GOAL", 12, 20, 1, 2);
    oled.display();
    delay(2000);
}

void search() {
    map_init();
    cx0 = 0;
    cy0 = 0;
    dir0 = DIR_N;
    pose_zero();
    visited[0][0] = true;
    sound_start();
    wait_motion(600);
    drive_mm(CELL_ENTRY_MM);
    if (blocked) {
        drive(0, 0);
        return;
    }
    NavState result = search_to(false, 0, 0, "SEARCH");
    if (result == NAV_DONE) {
        sound_goal();
        result = search_to(true, 0, 0, "RETURN");
    }
    if (result != NAV_DONE) {
        drive(0, 0);
        map_save();
        oled.clearDisplay();
        put_text(result == NAV_BLOCKED ? "STUCK" : "STOPPED", 16, 20, 1, 2);
        oled.display();
        wait_motion(1500);
        return;
    }
    bool verified = check_route(0);
    face(DIR_N);
    if (turnFault) {
        drive(0, 0);
        return;
    }
    distance_zero();
    propagate();
    trace(0, 0);
    map_save();
    drive(0, 0);
    sound_home();
    oled.clearDisplay();
    put_text(verified ? "MAZE SAVED" : "MAP SAVED", 4, 12, 1, 1);
    put_text("Steps: " + String(routeLen), 4, 28, 1, 1);
    put_text(verified ? "Select a run mode" : "Explore incomplete", 4, 44, 1, 1);
    oled.display();
    wait_motion(1800);
}

static bool replay_arcs(bool reverse, short int runSpeed) {
    if (routeLen == 0)
        return false;
    bool success = true;
    short int savedSpeed = pwm;
    short int savedBrake = brakeMs;
    pwm = runSpeed;
    baseL = runSpeed;
    baseR = runSpeed;
    brakeMs = (short int)constrain(
        (int)((float)savedBrake * runSpeed / (float)(savedSpeed > 0 ? savedSpeed : 100)), 0, 500);
    int step = 0;
    while (step < routeLen) {
        int idx = reverse ? (routeLen - 1 - step) : step;
        int dir = reverse ? opposed((int)route[idx]) : (int)route[idx];
        int count = 1;
        while (step + count < routeLen) {
            int nIdx = reverse ? (routeLen - 1 - (step + count)) : (step + count);
            int nDir = reverse ? opposed((int)route[nIdx]) : (int)route[nIdx];
            if (nDir != dir)
                break;
            count++;
        }
        int nextDir = -1;
        if (step + count < routeLen) {
            int nIdx = reverse ? (routeLen - 1 - (step + count)) : (step + count);
            nextDir = reverse ? opposed((int)route[nIdx]) : (int)route[nIdx];
        }
        face(dir);
        if (turnFault) {
            success = false;
            break;
        }
        dir0 = dir;
        ir_sample();
        drive_mm((float)count * pitch);
        if (blocked) {
            success = false;
            drive(0, 0);
            break;
        }
        cx0 += dx[dir0] * count;
        cy0 += dy[dir0] * count;
        if (!digitalRead(BTN_LEFT) || !digitalRead(BTN_RIGHT)) {
            success = false;
            drive(0, 0);
            delay(200);
            break;
        }
        if (nextDir >= 0) {
            int diff = (nextDir - dir + 4) & 3;
            if (diff == 1 || diff == 3) {
                arc_turn(diff == 1 ? 1 : 0, runSpeed);
                if (turnFault) {
                    success = false;
                    break;
                }
                dir0 = nextDir;
            } else {
                face(nextDir);
                if (turnFault) {
                    success = false;
                    break;
                }
                dir0 = nextDir;
            }
        }
        step += count;
    }
    drive(0, 0);
    wait_motion(300);
    pwm = savedSpeed;
    baseL = savedSpeed;
    baseR = savedSpeed;
    brakeMs = savedBrake;
    return success;
}

void run_fast() {
    int sX = origin_cell();
    int gx0, gy0, gx1, gy1;
    target_box(gx0, gy0, gx1, gy1);
    if (routeLen == 0) {
        if (!map_load()) {
            oled.clearDisplay();
            put_text("NO MAZE", 16, 10, 1, 2);
            put_text("EEPROM empty or", 2, 34, 1, 1);
            put_text("checksum bad", 2, 44, 1, 1);
            oled.display();
            delay(3000);
            return;
        }
        propagate();
        trace(sX, 0);
    }
    if (routeLen == 0) {
        oled.clearDisplay();
        put_text("NO PATH", 16, 10, 1, 2);
        put_text("Run FloodFill", 2, 38, 1, 1);
        put_text("sz:" + String(dim), 38, 50, 1, 1);
        oled.display();
        delay(3000);
        return;
    }
    oled.clearDisplay();
    put_text("FAST MODE", 8, 4, 1, 1);
    put_text("Steps:" + String(routeLen), 4, 16, 1, 1);
    put_text("Sp:" + String(pwm) + " Sz:" + String(dim), 4, 28, 1, 1);
    put_text("Place at START", 2, 40, 1, 1);
    put_text("R=GO   L=CANCEL", 2, 52, 1, 1);
    oled.display();
    while (1) {
        byte pb = button(BTN_RIGHT);
        if (pb == 1 || pb == 2)
            break;
        pb = button(BTN_LEFT);
        if (pb == 1 || pb == 2)
            return;
    }
    if (!route_valid()) {
        oled.clearDisplay();
        put_text("EXPLORE FIRST", 4, 18, 1, 1);
        oled.display();
        wait_motion(1800);
        return;
    }
    pose_zero();
    cx0 = sX;
    cy0 = 0;
    dir0 = DIR_N;
    short int sp1 = (short int)constrain((int)(pwm * 1.3f), 0, 255);
    short int sp2 = (short int)constrain((int)(pwm * 1.6f), 0, 255);
    oled.clearDisplay();
    put_text("FAST-A", 14, 4, 1, 1);
    put_text("SEG RUN", 8, 16, 1, 2);
    put_text("Spd:" + String(sp1) + " St:" + String(routeLen), 0, 40, 1, 1);
    oled.display();
    sound_phase(4);
    delay(800);
    if (!replay_blocks(false, sp1)) {
        drive(0, 0);
        return;
    }
    oled.clearDisplay();
    put_text("GOAL!", 22, 10, 1, 2);
    put_text(String(cx0) + "," + String(cy0), 34, 36, 1, 1);
    oled.display();
    delay(800);
    oled.clearDisplay();
    put_text("RETURN", 14, 16, 1, 2);
    oled.display();
    sound_phase(5);
    delay(400);
    if (!replay(true, sp1)) {
        drive(0, 0);
        return;
    }
    face(DIR_N);
    if (turnFault) {
        drive(0, 0);
        return;
    }
    distance_zero();
    cx0 = sX;
    cy0 = 0;
    dir0 = DIR_N;
    cx0 = sX;
    cy0 = 0;
    oled.clearDisplay();
    put_text("FAST-C", 14, 4, 1, 1);
    put_text("ARC RUN", 4, 16, 1, 2);
    put_text("Spd:" + String(sp2) + " St:" + String(routeLen), 0, 40, 1, 1);
    oled.display();
    sound_phase(6);
    delay(800);
    if (!replay_arcs(false, sp2)) {
        drive(0, 0);
        return;
    }
    for (int i = 0; i < 6; i++) {
        drive(70, -70);
        delay(100);
        drive(-70, 70);
        delay(100);
    }
    drive(0, 0);
    sound_done();
    oled.clearDisplay();
    put_text("FAST!", 10, 6, 1, 2);
    put_text("ArcSpd:" + String(sp2), 4, 30, 1, 1);
    put_text("Steps:" + String(routeLen), 4, 44, 1, 1);
    oled.display();
    delay(3000);
}

void run_direct(bool autoModerate) {
    int sX = origin_cell();
    if (routeLen == 0 && !map_load()) {
        oled.clearDisplay();
        put_text("NO MAZE", 16, 12, 1, 2);
        put_text("Solve the maze first", 0, 44, 1, 1);
        oled.display();
        delay(2200);
        return;
    }
    propagate();
    trace(sX, 0);
    if (routeLen == 0) {
        oled.clearDisplay();
        put_text("NO PATH", 16, 12, 1, 2);
        put_text("Map has no route", 4, 44, 1, 1);
        oled.display();
        delay(2200);
        return;
    }
    int gx0, gy0, gx1, gy1;
    target_box(gx0, gy0, gx1, gy1);
    short int modSpd = (short int)constrain((int)(pwm * 1.3f), 0, 255);
    short int fastSpd = (short int)constrain((int)(pwm * 1.7f), 0, 255);
    short int runSpd = modSpd;
    if (!autoModerate) {
        oled.clearDisplay();
        put_text("DIRECT RUN", 24, 2, 1, 1);
        put_text("to " + String(gx0) + "," + String(gy0) + "  " + String(routeLen) + " steps", 4,
                 14, 1, 1);
        put_text("Place at START", 4, 26, 1, 1);
        put_text("L=MOD " + String(modSpd) + "   R=FAST " + String(fastSpd), 0, 38, 1, 1);
        oled.fillRoundRect(0, 53, 128, 11, 3, 1);
        put_text("  C = cancel  ", 20, 55, 0, 1);
        oled.display();
        runSpd = 0;
        while (runSpd == 0) {
            if (button(BTN_LEFT))
                runSpd = modSpd;
            if (button(BTN_RIGHT))
                runSpd = fastSpd;
            if (button(BTN_CENTER))
                return;
        }
    }
    if (!route_valid()) {
        oled.clearDisplay();
        put_text("EXPLORE FIRST", 4, 18, 1, 1);
        oled.display();
        wait_motion(1800);
        return;
    }
    pose_zero();
    cx0 = sX;
    cy0 = 0;
    dir0 = DIR_N;
    oled.clearDisplay();
    put_text("GO", 50, 8, 1, 2);
    put_text("Spd:" + String(runSpd), 8, 34, 1, 1);
    put_text("L/R = STOP", 8, 46, 1, 1);
    oled.display();
    sound_start();
    wait_motion(600);
    bool completed = replay_blocks(false, runSpd);
    drive(0, 0);
    oled.clearDisplay();
    if (!completed) {
        put_text("STOPPED", 12, 8, 1, 2);
        put_text("at " + String(cx0) + "," + String(cy0), 30, 36, 1, 1);
        sound_error();
    } else {
        put_text("GOAL!", 24, 8, 1, 2);
        put_text(String(cx0) + "," + String(cy0) + "  spd " + String(runSpd), 18, 36, 1, 1);
        sound_goal();
    }
    oled.display();
    delay(2500);
}

void run_saved() {
    if (!map_load()) {
        oled.clearDisplay();
        put_text("NO MAZE", 16, 14, 1, 2);
        put_text("EEPROM empty or", 4, 38, 1, 1);
        put_text("checksum bad", 4, 48, 1, 1);
        oled.display();
        delay(2000);
        return;
    }
    if (routeLen == 0) {
        oled.clearDisplay();
        put_text("NO PATH", 16, 20, 1, 2);
        put_text("Solve first", 8, 44, 1, 1);
        oled.display();
        delay(2000);
        return;
    }
    int sX = origin_cell();
    cx0 = sX;
    cy0 = 0;
    dir0 = DIR_N;
    if (!route_valid()) {
        oled.clearDisplay();
        put_text("EXPLORE FIRST", 4, 18, 1, 1);
        oled.display();
        wait_motion(1800);
        return;
    }
    pose_zero();
    oled.clearDisplay();
    put_text("RUN PATH", 8, 16, 1, 2);
    put_text("Steps:" + String(routeLen), 14, 40, 1, 1);
    oled.display();
    delay(1500);
    bool completed = replay(false, pwm);
    oled.clearDisplay();
    put_text(completed ? " DONE! " : "STOPPED", 10, 20, 1, 2);
    oled.display();
    delay(1500);
}
