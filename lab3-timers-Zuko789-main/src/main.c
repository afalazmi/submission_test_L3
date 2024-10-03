/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 19, 2024
  * @brief   ECE 362 Lab 3 Student template
  ******************************************************************************
*/

/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "afalazmi";

/******************************************************************************
*/ #include "stm32f0xx.h"
#include <stdio.h>
#include <stdint.h>

// Global data structure
char disp[16]         = "Hello..."; // Increased size to 16
uint8_t col          = 0;
uint8_t mode         = 'A';
uint8_t thrust       = 0;
int16_t fuel         = 800;
int16_t alt          = 4500;
int16_t velo         = 0;

// Keymap is in font.S to match up what autotester expected
extern char keymap[]; // Ensure this is defined with at least 16 elements
extern uint8_t font[]; // Ensure this is defined appropriately

// Make it easier to access keymap
char* keymap_arr = keymap;

// Function Declarations
void enable_ports();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows, int col); // Corrected to accept two parameters
void handle_key(char key);
void setup_tim7();
void write_display();
void update_variables();
void setup_tim14();

// UART functions
void uart_init(void);
void uart_send_char(char c);
void uart_send_string(const char* str);
char uart_receive_char(void);

// Autotest functions
void internal_clock();
extern void check_wiring();
extern void autotest();
extern void fill_alpha();

// Function Prototypes for Interrupt Handlers
void USART2_IRQHandler(void);
void TIM7_IRQHandler(void);
void TIM14_IRQHandler(void);

// Buffer for UART Reception (optional for advanced handling)
#define UART_BUFFER_SIZE 128
char uart_buffer[UART_BUFFER_SIZE];
uint8_t uart_buffer_head = 0;
uint8_t uart_buffer_tail = 0;

int main(void) {
    internal_clock();

    // Initialize UART
    uart_init();

    // Uncomment when you are ready to test wiring.
    // check_wiring();

    // Uncomment when you are ready to test everything.
    autotest(); // Ensure this does not block. If it does, consider modifying autotest().

    enable_ports();

    // Comment out once you are checked off for fill_alpha
    fill_alpha();

    setup_tim7();
    setup_tim14();

    // Send a welcome message
    uart_send_string("STM32 Initialized. Type 'help' to see available commands.\n");

    for(;;) {
        // The main loop can remain empty or handle other non-blocking tasks
        // Enter low power sleep mode and Wait For Interrupt (WFI)
        asm("wfi");
    }
}

/**
 * @brief Initialize UART2 for serial communication
 */
void uart_init(void) {
    // Enable GPIOA clock (USART2 TX/RX are on PA2 and PA3)
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Configure PA2 (TX) and PA3 (RX) as Alternate Function
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2))); // Clear mode bits
    GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));    // Set to AF mode

    // Set Alternate Function to AF1 (USART2)
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4))); // Clear AFR bits
    GPIOA->AFR[0] |= (1 << (2 * 4)) | (1 << (3 * 4));        // Set AF1

    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure USART2: 115200 baud, 8 data bits, no parity, 1 stop bit
    // Assuming 16 MHz clock: Baud rate calculation
    // BRR = f_ck / baud
    // BRR = 16000000 / 115200 ≈ 138.888 ≈ 0x8A (for simplicity)
    USART2->BRR = 138; // Adjust based on actual clock if different

    // Enable USART2, TX and RX, and RXNE interrupt
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    // Enable USART2 interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief Send a single character via UART2
 * 
 * @param c Character to send
 */
void uart_send_char(char c) {
    while (!(USART2->ISR & USART_ISR_TXE)); // Wait until TX buffer is empty
    USART2->TDR = c;                        // Send character
}

/**
 * @brief Send a string via UART2
 * 
 * @param str Null-terminated string to send
 */
void uart_send_string(const char* str) {
    while (*str) {
        uart_send_char(*str++);
    }
}

/**
 * @brief Receive a single character via UART2 (Polling Method)
 * 
 * @return char Received character
 */
char uart_receive_char(void) {
    while (!(USART2->ISR & USART_ISR_RXNE)); // Wait until data is received
    return USART2->RDR;                       // Read received character
}

/**
 * @brief USART2 Interrupt Handler
 */
void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) { // Check if data is received
        char received = USART2->RDR;    // Read received character
        uart_send_char(received);        // Echo back the received character

        // Simple command handling
        if (received == '\r' || received == '\n') {
            uart_send_string("\nAvailable commands: help, status\n");
        } else if (received == 'h' || received == 'H') {
            uart_send_string("\nHelp: Available commands are 'help' and 'status'\n");
        } else if (received == 's' || received == 'S') {
            uart_send_string("\nStatus: All systems nominal.\n");
        }
    }
}

/**
 * @brief Enable the ports and configure pins as described
 *        in lab handout
 */
void enable_ports() {
    // Initialize Clock:
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
    // Set Pins B0-B10 as Output
    GPIOB->MODER &= ~(GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 |
                      GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 |
                      GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 |
                      GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 |
                     GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 |
                     GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 |
                     GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0);
    
    // Set Pins C4-C7 as outputs
    GPIOC->MODER &= ~(GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOC->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
    
    // Set Pins C0-C3 as inputs
    GPIOC->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    
    // Set Internal Pull Down Resistors for Pins C0-C3 with PUPDR
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 | GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1);  
}

/**
 * @brief Show a character c on column n
 *        of the segment LED display
 *
 * @param n Column number
 * @param c Character to display
 */
void show_char(int n, char c) {
    // Make sure n is in the right range (0-7)
    if(n < 0 || n > 7){
        return;  // Incorrect range
    }
    // Get the pattern from the font table
    uint8_t pattern = font[(int)c];
    // Clear PB0-PB7
    GPIOB->ODR &= ~(0xFF);
    // Set the bits of PB0-PB7 to the pattern for the given character
    GPIOB->ODR |= pattern;
    // Clear PB8-PB10 for the digit
    GPIOB->ODR &= ~(0x7 << 8);
    // Set PB8-PB10 based on the value of n to select the appropriate digit
    GPIOB->ODR |= (n << 8);
}

/**
 * @brief Drive the column pins of the keypad
 *        First clear the keypad column output
 *        Then drive the column represented by c
 *
 * @param c Column number (0-3)
 */
void drive_column(int c) {
    c = c & 0x03; // Lowest two bits of c extracted by ANDing with 0x03
    GPIOC->BSRR = (0xF << 4) << 16; // Clear bits 4-7 of GPIOC
    // Set bit corresponding to column c
    GPIOC->BSRR = (1 << (4 + c));
}

/**
 * @brief Read the rows value of the keypad
 *
 * @return int Binary value of row pins (e.g., 0bXXXX where X is 0 or 1)
 */
int read_rows() {
    // Read the row input pins (PC0-PC3) as a 4-bit value
    uint32_t row_inputs = GPIOC->IDR & 0xF;
    return row_inputs;
}

/**
 * @brief Convert the pressed key to character
 *        Use the rows value and the current col
 *        being scanned to compute an offset into
 *        the character map array
 *
 * @param rows Row value from read_rows
 * @param col  Current column being scanned
 * @return char The key pressed
 */
char rows_to_key(int rows, int col) { // Updated to accept two parameters
    // No keys pressed:
    if(rows == 0){
        return 0; // Indicate no key pressed
    }
    
    int row_pressed = -1; // Initialize to an invalid row
    
    if(rows & 0x01){
        row_pressed = 0; // Bottom Row
    }
    else if(rows & 0x02){
        row_pressed = 1; // Second row from bottom
    }
    else if(rows & 0x04){
        row_pressed = 2; // Third row from bottom
    }
    else if(rows & 0x08){
        row_pressed = 3; // Last row from bottom
    }
    else{
        return 0; // Invalid row, exit
    }
    
    int column = col & 0x3;  // Mask to get the two least significant bits (0-3 range)
    
    // Calculate the offset in the keymap (column * 4 + row)
    int offset = column * 4 + row_pressed;
    
    // Ensure offset is within bounds (assuming keymap_arr has at least 16 elements)
    if(offset < 0 || offset >= 16){
        return 0; // Invalid offset
    }
    
    // Use the offset to look up the key in the keymap array
    char key = keymap_arr[offset];
    return key;  // Return the corresponding key
}

/**
 * @brief Handle key pressed in the game
 *
 * @param key The pressed key
 */
void handle_key(char key) {
    if(key == 'A' || key == 'B' || key == 'D'){
        mode = key;
    }
    else if('0' <= key && key <= '9'){
        thrust = key - '0';
    }
}

/**
 * @brief Timer 7 Interrupt Service Routine (ISR)
 *        Handles periodic updates
 */
void TIM7_IRQHandler(void){
    // Check if the update interrupt flag is set
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag

        // Read the rows
        int rows = read_rows();
        // If a key is pressed, handle it
        if (rows != 0) {
            char key = rows_to_key(rows, col); // Pass both rows and col
            handle_key(key);
        }

        // Get the character from the disp array at position col
        char c = disp[col];

        // Show the character at column col
        show_char(col, c);
        // Increment col and wrap around if it exceeds 7
        col = (col + 1) % 8;
        drive_column(col);
    }
}

/**
 * @brief Setup timer 7 as described in lab handout
 */
void setup_tim7() {
    // Enable timer 7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Set timer 7 to generate interrupts at a specified interval
    // Prescaler (PSC) = 16000 - 1 for 1ms ticks (assuming 16 MHz clock)
    // Auto-Reload Register (ARR) = 1000 - 1 for 1 second intervals
    TIM7->PSC = 16000 - 1;  // Prescaler for 1ms ticks
    TIM7->ARR = 1000 - 1;   // Auto-reload value for 1 second intervals
    TIM7->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;    // Enable the timer

    // Enable TIM7 interrupt in NVIC
    NVIC_EnableIRQ(TIM7_IRQn);
}

/**
 * @brief Write the display based on game's mode
 */
void write_display() {
    if (mode == 'C'){
        // Display "Crashed" when mode is 'C'
        snprintf(disp, sizeof(disp), "Crashed");
    }
    else if (mode == 'L'){
        // Display "Landed" when mode is 'L'
        snprintf(disp, sizeof(disp), "Landed ");
    }
    else if (mode == 'A'){
        // Display altitude when mode is 'A', format is "ALt%5d"
        snprintf(disp, sizeof(disp), "ALt%5d", alt);
    }
    else if (mode == 'B'){
        // Display fuel level when mode is 'B', format is "FUEL %3d"
        snprintf(disp, sizeof(disp), "FUEL %3d", fuel);
    }
    else if (mode == 'D') {
        // Display speed/velocity when mode is 'D', format is "Spd %4d"
        snprintf(disp, sizeof(disp), "Spd %4d", velo);
    }
}

/**
 * @brief Game logic
 */
void update_variables() {
    fuel = fuel - thrust;
    if(fuel <= 0){
        fuel = 0;
        thrust = 0;
    }
    alt += velo;
    if(alt <= 0){
        if(velo > -10){
            mode = 'L';
        }
        else{
            mode = 'C';
        }
        return;
    }
    velo += thrust - 5;
}

/**
 * @brief Timer 14 Interrupt Service Routine (ISR)
 *        Handles keypad scanning
 */
void TIM14_IRQHandler(void){
    // Check if the update interrupt flag is set
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag

        // Perform timed tasks, e.g., scanning the keypad
        static int current_col = 0;  // Track the current column
        drive_column(current_col);   // Drive the column
        int rows = read_rows();      // Read the rows
        char key = rows_to_key(rows, current_col);  // Convert to key with both rows and col
        if (key) {
            handle_key(key);  // Handle the key if pressed
        }
        current_col = (current_col + 1) % 4;  // Cycle through columns (0-3)
    }
}

/**
 * @brief Setup timer 14 as described in lab handout
 */
void setup_tim14() {
    // Enable timer 14 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

    // Set timer 14 to generate interrupts for keypad scanning
    // Prescaler (PSC) = 16000 - 1 for 1ms ticks (assuming 16 MHz clock)
    // Auto-Reload Register (ARR) = 10 - 1 for 10ms intervals
    TIM14->PSC = 16000 - 1;  // Prescaler for 1ms ticks
    TIM14->ARR = 10 - 1;     // Auto-reload value for 10ms intervals
    TIM14->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    TIM14->CR1 |= TIM_CR1_CEN;    // Enable the timer

    // Enable TIM14 interrupt in NVIC
    NVIC_EnableIRQ(TIM14_IRQn); 
}