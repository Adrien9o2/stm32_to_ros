#pragma once

#include <memory>
#include <typeinfo>
#include <type_traits>

namespace SerialID
{
	constexpr uint8_t MSG_ACK = 0xFF;
	constexpr uint8_t MSG_START = 0xFE;
	constexpr uint8_t MSG_NO_START = 0xFD;
	constexpr uint8_t MSG_NO_ACK = 0xFC;
	constexpr uint8_t MSG_NO_ID = 0xFB;
	constexpr uint8_t MSG_NO_SIZE = 0xFA;
	constexpr uint8_t MSG_PRINT = 0x00;
	constexpr uint8_t MSG_MOTOR_SPEEDS = 0x01;
};


typedef struct _header_msg
{
	uint8_t MST_START;
	uint8_t MSG_ID;
	uint8_t MSG_LEN;
}header_msg;

typedef enum _msg_type
{
	header,
	payload,
	acknowledge
}msg_type;


class AbstractMsg
{
	public:
		AbstractMsg() = default;
		virtual ~AbstractMsg() {};
		virtual uint8_t* get_data() const noexcept= 0;
		virtual msg_type get_type() const noexcept= 0;
		virtual uint8_t get_data_size() const noexcept= 0;
};
class HeaderClass : public AbstractMsg
{
	static const msg_type header_msg_type = header;
	static const uint8_t header_size = 3;
	public:
		HeaderClass() = delete;
		HeaderClass(uint8_t msg_id, uint8_t msg_len): _data(new uint8_t[HeaderClass::header_msg_type])
		{
			_data[0] = SerialID::MSG_START;
			_data[1] = msg_id;
			_data[2] = msg_len;
		};
		~HeaderClass() = default;
		uint8_t* get_data() const noexcept override {return _data.get();};
		msg_type get_type() const noexcept override {return HeaderClass::header_msg_type;}
		uint8_t get_data_size() const noexcept override {return HeaderClass::header_size;}
	private:
		std::unique_ptr<uint8_t[]> _data;

};
class PayloadClass : public AbstractMsg
{
	static const msg_type payload_msg_type = payload;
	public:
		PayloadClass() = delete;
		template <typename T>
		PayloadClass(T* struct_or_plain_type_data, uint8_t data_size): _data_size(data_size), _data(new uint8_t[data_size])
		{
			static_assert(std::is_standard_layout<T>::value||std::is_arithmetic<T>::value ,"input data must be a plain struct or a built-in type");
			uint8_t* data_ptr = (uint8_t*) struct_or_plain_type_data;
			for( uint8_t i = 0; i < _data_size; i ++)
			{
				_data[i] = data_ptr[i];
			}

		};
		~PayloadClass()
		{
			delete _data;
		}
		uint8_t* get_data() const noexcept override {return _data;}
		msg_type get_type() const noexcept override {return PayloadClass::payload_msg_type;}
		uint8_t get_data_size() const noexcept override {return _data_size;}
	private:
		uint8_t _data_size;
		uint8_t* _data;
};
class AckMsg : public AbstractMsg
{
	static const msg_type incoming_msg_type = acknowledge;
	static const uint8_t incoming_msg_size = 3;
	public:
		AckMsg(): _data(new uint8_t[AckMsg::incoming_msg_size])
		{
			_data[0] = SerialID::MSG_NO_ACK;
			_data[1] = SerialID::MSG_NO_ID;
			_data[2] = SerialID::MSG_NO_SIZE;
		};
		~AckMsg() = default;
		AckMsg& operator=(const AckMsg& to_copy)
		{
			if (this != &to_copy)
			{
				if(to_copy._data)
				{
					for(int i = 0; i< AckMsg::incoming_msg_size; i ++)
					{
						_data[i] = to_copy._data[i];
					}
				}
				else
				{
					_data.reset();
				}


			}
			return *this;

		}
		uint8_t* get_data() const noexcept override {return _data.get();}
		msg_type get_type() const noexcept override {return AckMsg::incoming_msg_type;}
		uint8_t get_data_size() const noexcept override {return AckMsg::incoming_msg_size;}
	private:
		std::unique_ptr<uint8_t[]> _data;

};
