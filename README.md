# Slot-Machine

## Description

A FreeRTOS-based slot machine game running on the STM32F4 Discovery board. Press the on-board push-button to “spin.” Each spin randomly selects an LED. Collect all four unique LEDs to win. If you land on an LED you’ve already collected, you lose.

## Notes For The Instructor

- An alternate seed value for the PRNG is enabled in DEBUG builds for demonstration. This seed consistently produces a win on the first spin (useful for debugging).
- For true randomness, build and flash the RELEASE configuration, which initializes the PRNG seed using the hardware RNG peripheral. This ensures each game is genuinely unpredictable.
- To toggle between DEBUG and RELEASE in the STM32CubeIDE:
	1. Select the dropdown next to the green **Run** button.
	2. Choose **Slot-Machine Debug** or **Slot-Machine Release**
	3. If missing, select **Run Configurations** then double-click either **Slot-Machine Debug** or **Slot-Machine Release** under **STM32 C/C++ Application**.

## Win Condition & Animations

- **Win Condition**: Achieved when you collect all four unique LEDs without duplicates.
- **Winning Animation**: A rapid wheel of LEDs will be displayed. After 5 cycles, all LEDs flash before resetting to idle.
- **Losing Animation**: The red LED flashes 3 times, then all LEDs flash once before resetting the game.
- **Collect Animation**: When a new LED is collected, the LED(s) corresponding to the ones you’ve collected will flash 3 times (to confirm that LED has been added).
