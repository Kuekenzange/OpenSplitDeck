/**
 * @file display.c
 * @brief Display library implementation for SSD1306 OLED display
 */

#include "display.h"
#include "ui_Images.h"
#include <string.h>

LOG_MODULE_REGISTER(display_lib, LOG_LEVEL_ERR);

// Display device
static const struct device *display_dev = NULL;
static uint8_t controller_id = 0;

/**
 * @brief Initialize the display system
 */
int display_library_init(uint8_t ctrl_id)
{
    controller_id = ctrl_id;
    
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        return -ENODEV;
    }

    display_blanking_off(display_dev);
    return 0;
}

/**
 * @brief Get the current display status
 */
display_status_t display_get_status(void)
{
    if (display_dev && device_is_ready(display_dev)) {
        return DISPLAY_STATUS_READY;
    }
    return DISPLAY_STATUS_ERROR;
}

// Display buffer for drawing
static uint8_t display_buffer[DISPLAY_BUFFER_SIZE];

/**
 * @brief Clear the display buffer
 */
void display_clear(void)
{
    memset(display_buffer, 0, DISPLAY_BUFFER_SIZE);
}

/**
 * @brief Set a pixel in the display buffer
 */
void display_set_pixel(int16_t x, int16_t y, bool on)
{
    if (x >= 0 && x < DISPLAY_WIDTH && y >= 0 && y < DISPLAY_HEIGHT) {
        if (on) {
            display_buffer[x + (y / 8) * DISPLAY_WIDTH] |= (1 << (y & 7));
        } else {
            display_buffer[x + (y / 8) * DISPLAY_WIDTH] &= ~(1 << (y & 7));
        }
    }
}

/**
 * @brief Draw a simple letter at specified position - using original designs
 */
void display_draw_letter(char letter, int16_t x, int16_t y)
{
    static const uint8_t GLYPH_A[7] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t GLYPH_C[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    static const uint8_t GLYPH_E[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    static const uint8_t GLYPH_L[7] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
    static const uint8_t GLYPH_M[7] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    static const uint8_t GLYPH_N[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    static const uint8_t GLYPH_O[7] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t GLYPH_R[7] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    static const uint8_t GLYPH_T[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const uint8_t GLYPH_V[7] = {0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04};
    static const uint8_t GLYPH_X[7] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11};
    static const uint8_t GLYPH_Y[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};

    const uint8_t *glyph = NULL;
    char upper = letter;
    if (upper >= 'a' && upper <= 'z') {
        upper = (char)(upper - ('a' - 'A'));
    }

    switch (upper) {
        case 'A': glyph = GLYPH_A; break;
        case 'C': glyph = GLYPH_C; break;
        case 'E': glyph = GLYPH_E; break;
        case 'L': glyph = GLYPH_L; break;
        case 'M': glyph = GLYPH_M; break;
        case 'N': glyph = GLYPH_N; break;
        case 'O': glyph = GLYPH_O; break;
        case 'R': glyph = GLYPH_R; break;
        case 'T': glyph = GLYPH_T; break;
        case 'V': glyph = GLYPH_V; break;
        case 'X': glyph = GLYPH_X; break;
        case 'Y': glyph = GLYPH_Y; break;
        default:
            return;
    }

    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            if (glyph[row] & (1 << (4 - col))) {
                display_set_pixel(x + col, y + row, true);
            }
        }
    }
}

static void display_draw_word(int16_t x, int16_t y, const char *text)
{
    if (!text) {
        return;
    }

    int16_t cx = x;
    for (size_t i = 0; text[i] != '\0'; i++) {
        display_draw_letter(text[i], cx, y);
        cx += 6; // 5px glyph + 1px spacing
    }
}

/**
 * @brief Draw controller identification (without writing to display)
 */
void display_draw_controller_id(uint8_t ctrl_id)
{
    // Just draw to buffer, don't write to display yet
    // Position for 5x7 glyph in center
    int16_t letter_x = (DISPLAY_WIDTH - 5) / 2;
    int16_t letter_y = (DISPLAY_HEIGHT - 7) / 2;
    
    if (ctrl_id == 0) {
        display_draw_letter('R', letter_x, letter_y);
    } else {
        display_draw_letter('L', letter_x, letter_y);
    }
}

/**
 * @brief Write display buffer to screen (call only when content changes)
 */
void display_refresh_screen(void)
{
    if (!display_dev || !device_is_ready(display_dev)) {
        return;
    }
    
    struct display_buffer_descriptor desc = {
        .buf_size = DISPLAY_BUFFER_SIZE,
        .width = DISPLAY_WIDTH,
        .height = DISPLAY_HEIGHT,
        .pitch = DISPLAY_WIDTH
    };
    
    display_write(display_dev, 0, 0, &desc, display_buffer);
}

/**
 * @brief Main display update function
 */
void display_update(void)
{
    // This function is called by the display thread but shouldn't 
    // automatically redraw - let the screen state system handle it
    // Only refresh if we're in default mode
    static bool last_was_default = true;
    static bool initialized = false;
    
    if (!initialized) {
        display_show_status_screen();
        initialized = true;
        last_was_default = true;
    }
}

/**
 * @brief Enable/disable display blanking
 */
int display_set_blanking(bool blank)
{
    if (!display_dev) {
        return -ENODEV;
    }
    
    if (blank) {
        return display_blanking_on(display_dev);
    } else {
        return display_blanking_off(display_dev);
    }
}

void display_draw_hline(int16_t x, int16_t y, int16_t width)
{
    for (int16_t i = 0; i < width; i++) {
        display_set_pixel(x + i, y, true);
    }
}

/**
 * @brief Draw a vertical line
 */
void display_draw_vline(int16_t x, int16_t y, int16_t height)
{
    for (int16_t i = 0; i < height; i++) {
        display_set_pixel(x, y + i, true);
    }
}

/**
 * @brief Draw a rectangle
 */
void display_draw_rect(int16_t x, int16_t y, int16_t width, int16_t height, bool filled)
{
    if (filled) {
        for (int16_t i = 0; i < height; i++) {
            display_draw_hline(x, y + i, width);
        }
    } else {
        display_draw_hline(x, y, width);                    // Top
        display_draw_hline(x, y + height - 1, width);       // Bottom
        display_draw_vline(x, y, height);                   // Left
        display_draw_vline(x + width - 1, y, height);       // Right
    }
}

/**
 * @brief Draw a bitmap image from a byte array
 * 
 * @param x X position for top-left corner
 * @param y Y position for top-left corner
 * @param width Width of the bitmap in pixels
 * @param height Height of the bitmap in pixels
 * @param bitmap Pointer to bitmap data (1 bit per pixel, MSB first)
 */
void display_draw_bitmap(int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t* bitmap)
{
    if (!bitmap) return;
    
    for (int16_t row = 0; row < height; row++) {
        for (int16_t col = 0; col < width; col++) {
            // Calculate byte and bit position
            int16_t bit_index = row * width + col;
            int16_t byte_index = bit_index / 8;
            int16_t bit_position = 7 - (bit_index % 8); // MSB first
            
            // Check if pixel should be on
            bool pixel_on = (bitmap[byte_index] & (1 << bit_position)) != 0;
            
            // Draw the pixel
            display_set_pixel(x + col, y + row, pixel_on);
        }
    }
}

/**
 * @brief Draw a bitmap with transparency (0 = transparent, 1 = on, 2 = off)
 * 
 * @param x X position for top-left corner
 * @param y Y position for top-left corner  
 * @param width Width of the bitmap in pixels
 * @param height Height of the bitmap in pixels
 * @param bitmap Pointer to bitmap data (2 bits per pixel: 00=transparent, 01=off, 10=on, 11=on)
 */
void display_draw_bitmap_transparent(int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t* bitmap)
{
    if (!bitmap) return;
    
    for (int16_t row = 0; row < height; row++) {
        for (int16_t col = 0; col < width; col++) {
            // Calculate bit position (2 bits per pixel)
            int16_t pixel_index = row * width + col;
            int16_t byte_index = pixel_index / 4; // 4 pixels per byte
            int16_t bit_shift = 6 - ((pixel_index % 4) * 2); // 2 bits per pixel, MSB first
            
            // Extract 2-bit pixel value
            uint8_t pixel_value = (bitmap[byte_index] >> bit_shift) & 0x03;
            
            // Draw based on pixel value
            switch (pixel_value) {
                case 0x00: // Transparent - don't draw
                    break;
                case 0x01: // Off (black)
                    display_set_pixel(x + col, y + row, false);
                    break;
                case 0x02: // On (white)  
                case 0x03: // On (white)
                    display_set_pixel(x + col, y + row, true);
                    break;
            }
        }
    }
}
void display_draw_number(int16_t x, int16_t y, uint8_t number)
{
    // Simple 3x5 pixel font for numbers 0-9 - original design
    // Coordinates will be rotated by display_set_pixel
    
    const uint8_t font_3x5[][5] = {
        {0x7, 0x5, 0x5, 0x5, 0x7}, // 0
        {0x2, 0x2, 0x2, 0x2, 0x2}, // 1
        {0x7, 0x1, 0x7, 0x4, 0x7}, // 2
        {0x7, 0x1, 0x7, 0x1, 0x7}, // 3
        {0x5, 0x5, 0x7, 0x1, 0x1}, // 4
        {0x7, 0x4, 0x7, 0x1, 0x7}, // 5
        {0x7, 0x4, 0x7, 0x5, 0x7}, // 6
        {0x7, 0x1, 0x1, 0x1, 0x1}, // 7
        {0x7, 0x5, 0x7, 0x5, 0x7}, // 8
        {0x7, 0x5, 0x7, 0x1, 0x7}  // 9
    };
    
    if (number > 9) return;
    
    for (int row = 0; row < 5; row++) {
        for (int col = 0; col < 3; col++) {
            if (font_3x5[number][row] & (1 << (2 - col))) {
                display_set_pixel(x + col, y + row, true);
            }
        }
    }
}

/**
 * @brief Display analog values screen (simple vertical layout)
 */
void display_show_analog_screen(int16_t stick_x, int16_t stick_y, uint8_t trigger)
{
    display_clear();
    
    // Simple vertical layout for vertical screen
    // X value
    display_draw_letter('X', 2, 2);
    display_set_pixel(10, 4, true); // Colon
    
    uint16_t x_display = stick_x + 127;
    display_draw_number(12, 2, (x_display / 100) % 10);
    display_draw_number(16, 2, (x_display / 10) % 10);
    display_draw_number(20, 2, x_display % 10);
    
    // Y value
    display_draw_letter('Y', 2, 10);
    display_set_pixel(10, 12, true); // Colon
    
    uint16_t y_display = stick_y + 127;
    display_draw_number(12, 10, (y_display / 100) % 10);
    display_draw_number(16, 10, (y_display / 10) % 10);
    display_draw_number(20, 10, y_display % 10);
    
    // Trigger value
    display_draw_letter('T', 2, 18);
    display_set_pixel(10, 20, true); // Colon
    
    display_draw_number(12, 18, (trigger / 100) % 10);
    display_draw_number(16, 18, (trigger / 10) % 10);
    display_draw_number(20, 18, trigger % 10);
    
    display_refresh_screen();
}

/**
 * @brief Draw battery level indicator with voltage inside battery symbol
 */
void display_draw_battery_level(int16_t x, int16_t y, uint16_t voltage_mv)
{
    // First draw the battery outline
    display_draw_bitmap(x, y, 32, 32, bitmap_battery_sym);
    
    // Calculate battery percentage (assuming 3.2V-4.2V range)
    uint8_t percentage = 0;
    if (voltage_mv >= 4200) {
        percentage = 100;
    } else if (voltage_mv <= 3200) {
        percentage = 0;
    } else {
        // Linear mapping: 3200mV=0%, 4200mV=100%
        percentage = ((voltage_mv - 3200) * 100) / (4200 - 3200);
    }
    
    // Convert to 5 distinct levels with hysteresis to prevent flashing
    static uint8_t last_level = 0;
    uint8_t level = percentage / 20;  // 0-19%=0, 20-39%=1, 40-59%=2, 60-79%=3, 80-100%=4
    if (level > 4) level = 4;
    
    // Add hysteresis: only change level if we're clearly in the new range
    if (level > last_level) {
        // Going up: need to be at least 2% into the new level
        if (percentage < (level * 20 + 2)) {
            level = last_level;
        }
    } else if (level < last_level) {
        // Going down: need to be at least 2% below the old level  
        if (percentage > ((last_level * 20) - 2)) {
            level = last_level;
        }
    }
    last_level = level;
    
    // Battery bitmap is horizontal: terminal at top, body in middle, closed at bottom
    // Screen is vertical so adjust Y coordinate (move "up" on vertical screen)
    // Interior spans roughly x+10 to x+21 (width) and y+2 to y+25 (height)
    
    if (level > 0) {
        // Level 1 (20%): Top horizontal segment  
        for (int16_t fill_y = y + 2; fill_y <= y + 6; fill_y++) {
            for (int16_t fill_x = x + 10; fill_x <= x + 21; fill_x++) {
                display_set_pixel(fill_x, fill_y, true);
            }
        }
    }
    
    if (level > 1) {
        // Level 2 (40%): Second horizontal segment from top
        for (int16_t fill_y = y + 7; fill_y <= y + 11; fill_y++) {
            for (int16_t fill_x = x + 10; fill_x <= x + 21; fill_x++) {
                display_set_pixel(fill_x, fill_y, true);
            }
        }
    }
    
    if (level > 2) {
        // Level 3 (60%): Middle horizontal segment
        for (int16_t fill_y = y + 12; fill_y <= y + 16; fill_y++) {
            for (int16_t fill_x = x + 10; fill_x <= x + 21; fill_x++) {
                display_set_pixel(fill_x, fill_y, true);
            }
        }
    }
    
    if (level > 3) {
        // Level 4 (80%): Fourth horizontal segment from top
        for (int16_t fill_y = y + 17; fill_y <= y + 21; fill_y++) {
            for (int16_t fill_x = x + 10; fill_x <= x + 21; fill_x++) {
                display_set_pixel(fill_x, fill_y, true);
            }
        }
    }
    
    if (level >= 4) {
        // Level 5 (100%): Bottom horizontal segment
        for (int16_t fill_y = y + 22; fill_y <= y + 26; fill_y++) {
            for (int16_t fill_x = x + 10; fill_x <= x + 21; fill_x++) {
                display_set_pixel(fill_x, fill_y, true);
            }
        }
    }
}

/**
 * @brief Display controller status screen
 */
/**
 * @brief Display controller status screen with battery level
 */
void display_show_status_screen_with_battery(uint16_t battery_mv)
{
    display_clear();
    
    // Draw the appropriate controller bitmap (96x32 pixels)
    if (controller_id == 0) {
        // Right controller (ID 0)
        display_draw_bitmap(0, 0, 96, 32, bitmap_RightController_Tall);
    } else {
        // Left controller (ID 1) 
        display_draw_bitmap(0, 0, 96, 32, bitmap_LeftController_Tall);
    }

    // Draw battery level indicator at position (96, 0)
    display_draw_battery_level(96, 0, battery_mv);
    
    // Write to display only once
    display_refresh_screen();
}

void display_show_status_screen(void)
{
    display_clear();
    
    // Draw the appropriate controller bitmap (32x32 pixels)
    // Center it on the display (128x32, so center is at x=48, y=0)
    if (controller_id == 0) {
        // Right controller (ID 0)
        display_draw_bitmap(0, 0, 96, 32, bitmap_RightController_Tall);
    } else {
        // Left controller (ID 1) 
        display_draw_bitmap(0, 0, 96, 32, bitmap_LeftController_Tall);
    }

    display_draw_bitmap(96, 0, 32, 32, bitmap_battery_sym);
    
    // Write to display only once
    display_refresh_screen();
}

/**
 * @brief Display calibration screen with progress
 */
void display_show_calibration_screen(uint8_t phase, uint8_t progress)
{
    display_clear();

    const char *title = "CAL";
    int title_width = (int)strlen(title) * 6 - 1;
    int16_t title_x = (DISPLAY_WIDTH - title_width) / 2;
    display_draw_word(title_x, 2, title);

    const char *phase_text = (phase == 0) ? "CENTER" : "MOVE";
    int phase_width = (int)strlen(phase_text) * 6 - 1;
    int16_t phase_x = (DISPLAY_WIDTH - phase_width) / 2;
    display_draw_word(phase_x, 12, phase_text);

    // Side marker to confirm LEFT/RIGHT selection on this screen
    display_draw_letter((controller_id == 0) ? 'R' : 'L', 120, 2);
    
    // Draw progress bar
    int16_t bar_width = (progress * 100) / 100;  // Scale to 100 pixels wide
    display_draw_rect(14, 24, 100, 6, false);  // Border
    if (bar_width > 0) {
        display_draw_rect(15, 25, bar_width, 4, true);  // Fill
    }
    
    display_refresh_screen();
}
