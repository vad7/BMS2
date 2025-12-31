# BMS2-Microart
Several BMS gate to Microart inverter

Добавлен симыольный LCD дисплей 20x4 символов.<br>
К инвертору МАП производителя Микроарт можно подключить BMS для балансировки ячеек аккумулятора как их собственного производства, так и других производителей. 
Что бы МАП видел BMS как свои они должны быть подключены по шине I2C.
<br><br>
В данном репозитории представлен I2C гейт для нескольких балансиров RS485 - Микроарт. Активные балансир JK-DZ11-B2A24S RS485.<br>
Количество BMS - #define BMS_NUM_MAX<br>
<br>
<br>
Выходной программный UART со скоростью 333 бод, для оповещения об ошибках и состоянии BMS другому контроллеру, например, к контроллеру ТН - https://github.com/vad7/Control<br>
Формат данных:<br>
uint8_t  BMS_num;<br>
--- структура по каждому BMS<br>
uint8_t  last_status;	// bms_flags (b7=on/off, b6=balancing) + last_error ERR_BMS_*<br>
uint8_t  bms_min_string;<br>
uint8_t  bms_max_string;<br>
uint16_t bms_min_cell_mV;<br>
uint16_t bms_max_cell_mV;<br>
int32_t  bms_total_mV;<br>
---<br>
CRC16 (Modbus)<br>
<br>
ERR_BMS_*:<br>
ERR_BMS_Ok = 0,<br>
ERR_BMS_NotAnswer = 1,<br>
ERR_BMS_Read = 2,<br>
ERR_BMS_Config = 3,<br>
ERR_BMS_Resistance = 4,<br>
ERR_BMS_Cell_Low = 5,<br>
ERR_BMS_Cell_High = 6,<br>
ERR_BMS_Cell_DeltaMax = 7<br>
