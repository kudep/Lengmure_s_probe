#define PIN_RELAY_1 2
#define PIN_RELAY_2 3
#define PIN_RELAY_3 4
#define PIN_RELAY_4 5

#define PIN_DAC 6
#define PIN_ADC_V 0
#define PIN_ADC_I 1
// Эта задержка для защиты от продолжительной подачи высокого напряжения в результате долгого ответа сервера
#define TRANS_DELAY_MS 10000
#define INIT_STATE_TRANS false

byte num_cmnd, DAC_init_volt, DAC_volt, DAC_step, DAC_count_step, DAC_delay, DAC_repeat, present_step, num;
int16_t ADC_V, ADC_I, data[4];
bool state_trans;
bool trans_ask1,trans_ask2;//переменная регистрирует ответ сервера на передачу



void setup() {

	pinMode(PIN_RELAY_1, OUTPUT);
	pinMode(PIN_RELAY_2, OUTPUT);
	pinMode(PIN_RELAY_3, OUTPUT);
	pinMode(PIN_RELAY_4, OUTPUT);
	pinMode(PIN_DAC, OUTPUT);



	relay_off();

	TCCR1B = TCCR1B & 0xf8 | 0x01; // увеличение частоты ШИМ до 62500 Гц http://kazus.ru/forums/showthread.php?p=768845
	volt_out(0);

	Serial.begin(9600, SERIAL_8E1);
	num_cmnd = DAC_repeat = num=0;
	state_trans = INIT_STATE_TRANS;
	trans_ask1 = false;//Ответ не получен
	trans_ask2 = false;//Ответ не верен
	//debag();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debag
}
void loop() {
	init_cmd();
	driver();
}
void init_cmd()
{
	switch (num_cmnd)
	{
	case 0: //Ожидание байта команды
		if (Serial.available()) num_cmnd++;
		return;
	case 1://Проверка первого байта
		if ('G' == (char)Serial.peek())
		{
			num_cmnd++;
			relay_off();
			volt_out(0);
			Serial.read();
		}
		else
		{
			trans_ask();
			Serial.flush();
			num_cmnd = 0;
		}
		return;

	case 2://Ожидание следующего байта команды, отвечающего за реле
		if (Serial.available())
		{
			num_cmnd++;
		}
		return;
	case 3://Включение необходимых реле
		if (relay_on(serial_read()))
		{
			num_cmnd++;
		}

		else
		{
			Serial.flush();
			num_cmnd = 0;
		}
		return;

	case 4://Ожидание следующего байта команды, отвечающего за начальное напряжения
		if (Serial.available())
		{
			num_cmnd++;
		}
		return;
	case 5://Определение начального напряжения
		init_volt(serial_read());
		num_cmnd++;
		return;

	case 6://Ожидание следующего байта команды, отвечающего за диапазон напряжения
		if (Serial.available())
		{
			num_cmnd++;
		}
		return;
	case 7://Определение диапазона напряжения
		init_range(serial_read());
		num_cmnd++;
		return;

	case 8://Ожидание следующего байта команды, отвечающего за количество шагов
		if (Serial.available())
		{
			num_cmnd++;
		}
		return;
	case 9://Определение количество шагов
		init_step(serial_read());
		num_cmnd++;
		return;

	case 10://Ожидание следующего байта команды, отвечающего за задержку
		if (Serial.available())
		{
			num_cmnd++;
		}
		return;
	case 11://Определение задержки
		init_delay(serial_read());
		num_cmnd++;
		return;

	case 12://Ожидание следующего байта команды, отвечающего за количество повторений
		if (Serial.available())
		{
			num_cmnd++;
		}
		return;
	case 13://Определение количество повторений
		init_repeat(serial_read());
		//debag();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debag
		num_cmnd=0;
		num = 0;
		state_trans = INIT_STATE_TRANS;
		return;
	default://Обнуление счетчика команд
		num_cmnd = 0;
		return;
	}
}
byte serial_read()
{
	return Serial.read();// -'0'; //Убрать 0 из конечной версии//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debag

}
void relay_off()
{
	digitalWrite(PIN_RELAY_1, LOW);
	digitalWrite(PIN_RELAY_2, LOW);
	digitalWrite(PIN_RELAY_3, LOW);
	digitalWrite(PIN_RELAY_4, LOW);

}
bool relay_on(byte cmnd)
{
	if (cmnd & 15){
		if (cmnd & 1) digitalWrite(PIN_RELAY_1, HIGH);
		if (cmnd & 2) digitalWrite(PIN_RELAY_2, HIGH);
		if (cmnd & 4) digitalWrite(PIN_RELAY_3, HIGH);
		if (cmnd & 8) digitalWrite(PIN_RELAY_4, HIGH);
		return true;
	}
	return false;

}
void init_volt(byte cmnd)
{
	DAC_init_volt = cmnd;
}
void init_range(byte cmnd)
{
	DAC_step = cmnd;
}
void init_step(byte cmnd)
{
	DAC_count_step = cmnd;
	DAC_step /= DAC_count_step;

}
void init_delay(byte cmnd)
{
	DAC_delay = cmnd;
}
void init_repeat(byte cmnd)
{
	DAC_repeat = cmnd;
}
void driver()
{

	static unsigned long time_control = 0;
	switch (num)
	{
	case 0:
		if (DAC_repeat)
		{
			DAC_volt = DAC_init_volt;
			volt_out(DAC_volt);
			time_check(&time_control);
			DAC_repeat--;
			num++;
			DAC_volt += DAC_step;
			present_step = 0;
		}
		return;
	case 1:
		if (time_wait(&time_control, DAC_delay)) num++;
		
		return;
	case 2:
		detect();
		if (present_step < DAC_count_step)
		{
			volt_out(DAC_volt);
			time_check(&time_control);
			present_step++;
			DAC_volt += DAC_step;
			num++;

		}
		else
			init_driver();
		return;
	case 3: 
		if (transiv()) num = 1;
		return;
	default://Обнуление счетчика
		num = 0;
		state_trans = INIT_STATE_TRANS;
		return;
		
		
	}

}
void volt_out(byte value)
{
	analogWrite(PIN_DAC,255 - value);
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!debag
	//Serial.print("Out voltage ");
	//Serial.println(value, DEC);
}
void time_check(unsigned long *time_control)
{
	(*time_control) = millis();
	//Serial.print("Check time ");
	//Serial.println(time_control, DEC);
}
bool time_wait(unsigned long *time_control, uint16_t delay)
{
	return (millis() - (*time_control))>delay * 10;
}
void detect()
{
	if (state_trans)
	{
		data[0] = ADC_read(PIN_ADC_V);
		data[1] = ADC_read(PIN_ADC_I);
		state_trans = false;
	}
	else
	{
		data[2] = ADC_read(PIN_ADC_V);
		data[3] = ADC_read(PIN_ADC_I);
		state_trans = true;
	}
}
bool transiv()
{
	int16_t data_p[4];
	static unsigned long time_control=0;
	static bool trans = false;
	if (state_trans)return true;

	if (trans)
	{
		if (time_wait(&time_control, TRANS_DELAY_MS)) //на случай потерянного ответа от сервера - обнуление всех команд
		{
			trans = false;
			init_driver();
			return false;
		}
		if (trans_ask1)
		{
			trans = false;
			if (trans_ask2)
			{
				trans = false;
				return true;
			}
		}
		return false;
	}
	else
	{
		Serial.print('R');
		for (byte i = 0; i < 4; i++)
		{
			Serial.write(data[i] >> 2);
			data_p[i] = data[i] & 3;
		}
		Serial.write((data_p[0]) | (data_p[1] << 2) | (data_p[2] << 4) | (data_p[3] << 6));
		trans = true;
		time_check(&time_control);
		trans_ask_wait();
		return false;
	}
	
}
int ADC_read(byte pin)
{
	unsigned int read_sum = 0;
	for (int i = 0; i < 16; i++)
		read_sum += analogRead(pin);
	return read_sum >> 4;
}
void init_driver(void)
{
	num = 0;
	present_step = 0;
	volt_out(0);
	state_trans = true;

}
void trans_ask_wait(void)
{
	trans_ask1 = false;
	trans_ask2 = false;
}
void trans_ask(void)
{
	trans_ask1 = true;
	if ('F' == (char)Serial.read())trans_ask2 = true;
}
void debag()
{
	Serial.print("num_cmnd ");
	Serial.println(num_cmnd, DEC);
	Serial.print("DAC_init_volt ");
	Serial.println(DAC_volt, DEC);
	Serial.print("DAC_count_step ");
	Serial.println(DAC_count_step, DEC);
	Serial.print("DAC_step ");
	Serial.println(DAC_step, DEC);
	Serial.print("DAC_delay ");
	Serial.println(DAC_delay, DEC);
	Serial.print("DAC_repeat ");
	Serial.println(DAC_repeat, DEC);
	Serial.println();
	Serial.println();
	Serial.println();
}