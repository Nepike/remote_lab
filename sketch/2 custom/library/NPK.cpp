#include "NPK.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);


uint16_t DataPack::calculate_crc(const uint8_t* data, size_t data_size){
	CRC16 crc;
	crc.reset();
	for (size_t i = 0; i < data_size; i++) {
		crc.add(data[i]);
	}
	return crc.calc();
}

DataPack::DataPack(Command command): command_(command), args_count_(0), args_types_(new ArgType[MAX_ARGS_COUNT]()), payload_length_(0), payload_(new uint8_t[MAX_PAYLOAD_SIZE]()), current_payload_index_(0), is_valid_(true) {}

DataPack::DataPack(SoftwareSerial& serial) : args_types_(new ArgType[MAX_ARGS_COUNT]()), payload_(new uint8_t[MAX_PAYLOAD_SIZE]()), current_payload_index_(0), is_valid_(false) {

    const uint32_t TIMEOUT = 100; // Таймаут 100 мс
    uint8_t data[MAX_PACK_SIZE];
    size_t data_size = 0;

    // Лямбда для чтения байта с таймаутом
    auto read_byte = [&](uint8_t& byte) -> bool {
        uint32_t start = millis();
        while (millis() - start < TIMEOUT) {
            if (serial.available()) {
                byte = serial.read();
                return true;
            }
        }
        return false;
    };

    // Чтение стартового маркера
    uint8_t start_bytes[2];
    if (!read_byte(start_bytes[0])) return;
    if (!read_byte(start_bytes[1])) return;
    
    if (start_bytes[0] != 0xAA || start_bytes[1] != 0x55) {
        return;
    }
    
    memcpy(data, start_bytes, 2);
    data_size = 2;

    // Чтение команды
    uint8_t cmd_byte;
    if (!read_byte(cmd_byte)) return;
    command_ = static_cast<Command>(cmd_byte);
    data[data_size++] = cmd_byte;

    // Чтение количества аргументов
    uint8_t args_count;
    if (!read_byte(args_count)) return;
    data[data_size++] = args_count;
    
    if (args_count > MAX_ARGS_COUNT) return;
    args_count_ = args_count;

    // Чтение типов аргументов
    for (size_t i = 0; i < args_count_; i++) {
        uint8_t type_byte;
        if (!read_byte(type_byte)) return;
        if (type_byte < 0x01 || type_byte > 0x06) return;
        
        args_types_[i] = static_cast<ArgType>(type_byte);
        data[data_size++] = type_byte;
    }

    // Чтение длины полезной нагрузки
    uint8_t payload_len;
    if (!read_byte(payload_len)) return;
    data[data_size++] = payload_len;
    
    if (payload_len > MAX_PAYLOAD_SIZE) return;
    payload_length_ = payload_len;

    // Чтение полезной нагрузки
    for (size_t i = 0; i < payload_length_; i++) {
        if (!read_byte(payload_[i])) return;
        data[data_size++] = payload_[i];
    }

    // Чтение CRC
    uint8_t crc_bytes[2];
    if (!read_byte(crc_bytes[0])) return;
    if (!read_byte(crc_bytes[1])) return;
    
    uint16_t received_crc = (crc_bytes[0] << 8) | crc_bytes[1];
    uint16_t calculated_crc = calculate_crc(data, data_size);

    // Проверка целостности данных
    is_valid_ = (received_crc == calculated_crc);

    // Очистка буфера в случае ошибки
    if (!is_valid_) {
        while (serial.available()) serial.read();
    }
}

DataPack::DataPack(DataPack&& other) noexcept
: command_(other.command_), args_count_(other.args_count_), args_types_(other.args_types_), payload_length_(other.payload_length_), payload_(other.payload_), current_payload_index_(other.current_payload_index_), is_valid_(other.is_valid_) {
	other.args_types_ = nullptr;
	other.payload_ = nullptr;
}

DataPack::~DataPack() {
	delete[] args_types_;
	delete[] payload_;
}

size_t DataPack::get_serialized_size() const {
	return MIN_PACK_SIZE + args_count_ + payload_length_;
}

uint8_t* DataPack::get_serialized_data() const {
	size_t buffer_length = get_serialized_size();
	uint8_t* buffer = new uint8_t[buffer_length];

	// Start Marker
	buffer[0] = 0xAA;
	buffer[1] = 0x55;

	// Command
	buffer[2] = static_cast<uint8_t>(command_);

	// Data Types Flags
	buffer[3] = static_cast<uint8_t>(args_count_);
	for (size_t i = 0; i < args_count_; i++){
		buffer[4+i] = static_cast<uint8_t>(args_types_[i]);
	}

	// Payload
	buffer[4 + args_count_] = static_cast<uint8_t>(payload_length_);
	for (size_t i = 0; i < payload_length_; i++){
		buffer[5 + args_count_ + i] = payload_[i];
	}

	// CRC
	uint16_t crc_value = calculate_crc(buffer, buffer_length-2);
	buffer[buffer_length-2] = static_cast<uint8_t>(crc_value >> 8);
	buffer[buffer_length-1] = static_cast<uint8_t>(crc_value & 0xFF);

	return buffer;
}

template<>
bool DataPack::get_next_val<bool>() {
  return static_cast<bool>(payload_[current_payload_index_++]);
}

template<>
uint8_t DataPack::get_next_val<uint8_t>() {
  return payload_[current_payload_index_++];
}

template<>
int16_t DataPack::get_next_val<int16_t>() {
  int16_t result = static_cast<int16_t>((payload_[current_payload_index_++] << 8) | payload_[current_payload_index_++]);
  return result;
}

template<>
uint16_t DataPack::get_next_val<uint16_t>() {
  uint16_t result = static_cast<uint16_t>((payload_[current_payload_index_++] << 8) | payload_[current_payload_index_++]);
  return result;
}

template<>
float DataPack::get_next_val<float>() {
  float result;
  memcpy(&result, &payload_[current_payload_index_], 4);
  current_payload_index_ += 4;
  return result;
}

template<>
const char* DataPack::get_next_val<const char*>() {
  uint8_t str_len = payload_[current_payload_index_++];
  char* result = new char[str_len + 1];
  memcpy(result, &payload_[current_payload_index_], str_len);
  result[str_len] = '\0';
  current_payload_index_ += str_len;
  return result;
}

void DataPack::append_arg(bool value) {
	if (args_count_ >= MAX_ARGS_COUNT || payload_length_ + 1 > MAX_PAYLOAD_SIZE) return;
	args_types_[args_count_++] = ArgType::BOOL;
	payload_[payload_length_++] = value ? 1 : 0;
}

void DataPack::append_arg(uint8_t value) {
	if (args_count_ >= MAX_ARGS_COUNT || payload_length_ + 1 > MAX_PAYLOAD_SIZE) return;
	args_types_[args_count_++] = ArgType::UINT8;
	payload_[payload_length_++] = value;
}

void DataPack::append_arg(int16_t value) {
	if (args_count_ >= MAX_ARGS_COUNT || payload_length_ + 2 > MAX_PAYLOAD_SIZE) return;
	args_types_[args_count_++] = ArgType::INT16;
	payload_[payload_length_++] = static_cast<uint8_t>(value >> 8);
	payload_[payload_length_++] = static_cast<uint8_t>(value & 0xFF);
}

void DataPack::append_arg(uint16_t value) {
	if (args_count_ >= MAX_ARGS_COUNT || payload_length_ + 2 > MAX_PAYLOAD_SIZE) return;
	args_types_[args_count_++] = ArgType::UINT16;
	payload_[payload_length_++] = static_cast<uint8_t>(value >> 8);
	payload_[payload_length_++] = static_cast<uint8_t>(value & 0xFF);
}

void DataPack::append_arg(float value) {
	if (args_count_ >= MAX_ARGS_COUNT || payload_length_ + 4 > MAX_PAYLOAD_SIZE) return;
	args_types_[args_count_++] = ArgType::FLOAT;
	const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
	for (int i = 0; i < 4; i++) {
		payload_[payload_length_++] = bytes[i];
	}
}

void DataPack::append_arg(const char* value) {
	size_t str_len = strlen(value);
	if (str_len > MAX_STRING_SIZE) str_len = MAX_STRING_SIZE;
	size_t required = 1 + str_len;
	if (args_count_ >= MAX_ARGS_COUNT || payload_length_ + required > MAX_PAYLOAD_SIZE) return;
	args_types_[args_count_++] = ArgType::STRING;
	payload_[payload_length_++] = static_cast<uint8_t>(str_len);
	memcpy(&payload_[payload_length_], value, str_len);
	payload_length_ += str_len;
}

bool DataPack::is_valid() const { return is_valid_; }

size_t DataPack::get_args_count() const {return args_count_;}


DataPack DataPack::perform_command() {
	DataPack response(Command::RESPONSE);

	if (!is_valid()) {
		response.is_valid_ = false;
		return response;
	}

	switch(command_){
		case Command::RESPONSE: {
			response.is_valid_ = false;
			return response;
		}
		// case Command::EXAMPLE: {
		//   //Функия return_type example(type1 arg1, type2 arg2, ...) - есть в поле видимости и имеет соответствующий индекс в Command
		//   // Из-за невозмжности отключить NRVO придется явно сохранять возвращаемые значения в переменных: 
		//   auto arg1 = get_next_val<type1>();
		//   auto arg2 = get_next_val<type2>();
		//   ...
		//   return_type return_value = example(arg1, arg2, ...);
		//   // Заполнение response
		//   return response;
		// }
		case Command::BLINK: {
			pinMode(LED_BUILTIN, OUTPUT);
			digitalWrite(LED_BUILTIN, HIGH);
			delay(1000);
			digitalWrite(LED_BUILTIN, LOW);
			delay(1000);
			break;
		}
		case Command::LCD: {
			lcd.clear();
			lcd.setCursor(0,0);
			auto arg = get_next_val<const char*>();
			lcd.print(arg);
			break;
		}
		case Command::GETRANGE: {
			int max_length = 0; //надо померить
			int min_length = 1023; //надо померить
			int S1 = 6;
			int sig = analogRead(S1);
			sig = map(sig, 0, 1023,max_length, min_length);
			String s = String(sig);
			response.append_arg(s.c_str());

			break;
		}
		case Command::GO: {
			int IN1 = 9;
			int IN2 = 8;
			int IN3 = 7; 
			int IN4 = 6; 
			int EN1 = 10; 
			int EN2 = 5; 
			uint8_t direction = get_next_val<uint8_t>();
			uint8_t speed = get_next_val<uint8_t>();
			uint16_t time = get_next_val<uint16_t>();

			analogWrite (EN1, 0); 
			analogWrite (EN2, 0);
			if (time == 0) {
				break;
			}

			switch(direction) {
				case 1 : {
					digitalWrite (IN2, HIGH); 
					digitalWrite (IN1, LOW); 
					digitalWrite (IN4, HIGH); 
					digitalWrite (IN3, LOW);
					break;
				}
				case 3: {
					digitalWrite (IN1, HIGH); 
					digitalWrite (IN2, LOW); 
					digitalWrite (IN3, HIGH); 
					digitalWrite (IN4, LOW);
					break;
				}
				case 4: {
					digitalWrite (IN1, HIGH); 
					digitalWrite (IN2, LOW); 
					digitalWrite (IN4, HIGH); 
					digitalWrite (IN3, LOW);
					break;
				}
				case 2: {
					digitalWrite (IN2, HIGH); 
					digitalWrite (IN1, LOW); 
					digitalWrite (IN3, HIGH); 
					digitalWrite (IN4, LOW);
					break;
				}
				default: {return response;}
			}

			analogWrite(EN1, speed); 
			analogWrite(EN2, speed);

			if (time == 1) {
				break;
			}

			long l = millis();
			while (true) {
				if (millis() - l > time) {
					break;
				}
			}
			analogWrite (EN1, 0); 
			analogWrite (EN2, 0);
			break;
		}
		default: {
			return response;
		};
	}
	return response;
}





