#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <new>

#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <Servo.h>

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))
#define RCL_MS_TO_NS(ms) ((ms) * 1000000ULL)
#define RCCHECK(fn) { rcl_ret_t temp_rc = (fn); if ((temp_rc) != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = (fn); (void) temp_rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > (MS)) { X; init = uxr_millis(); } \
} while (0)

static const uint32_t SERIAL_BAUDRATE = 115200;
static const uint32_t EXECUTOR_TIMEOUT_MS = 5;
static const uint32_t BUTTON_SAMPLE_PERIOD_MS = 20;
static const uint32_t INPUT_DEBOUNCE_MS = 30;
static const uint32_t THERMAL_SAMPLE_PERIOD_MS = 150;
static const uint32_t AGENT_PING_PERIOD_MS = 2000;
static const uint32_t AGENT_PING_TIMEOUT_MS = 100;
static const uint8_t LED_DATA_PIN = 30;
static const uint16_t LED_COUNT = 24;
static const uint8_t LED_BRIGHTNESS = 200;

enum DigitalInputKind : uint8_t {
  MOMENTARY_INPUT = 0,
  LATCHED_INPUT,
};

struct DigitalChannelState {
  uint8_t pin;
  bool last_state;
  unsigned long last_change_ms;
  DigitalInputKind kind;
};

struct DigitalInputConfig {
  uint8_t pin;
  DigitalInputKind kind;
};

static const DigitalInputConfig DIGITAL_INPUTS[] = {
  {28, MOMENTARY_INPUT},  // D28 button
  {29, LATCHED_INPUT},    // D29 switch
  {16, LATCHED_INPUT},    // D16 switch
  {15, MOMENTARY_INPUT},  // D15 button
};
static const char * const BUTTON_TOPIC = "/arduino/buttons";
static const char * const PY_LEDS_TOPIC = "/arduino/py_leds";
static const char * const THERMO_TOPIC = "/arduino/thermovisor";

struct ServoChannelConfig {
  uint8_t pin;
};

static const ServoChannelConfig SERVO_CHANNELS[] = {
  {2},
  {3},
  {4},
  {5},
  {6},
  {7},
};

static const char * const SERVO_COMMAND_TOPIC = "/arduino/servos";

static const uint16_t SERVO_MIN_PULSE_US = 1000;
static const uint16_t SERVO_MAX_PULSE_US = 2000;
static const uint16_t SERVO_CENTER_PULSE_US = 1500;

static const int16_t SERVO_COMMAND_MIN = 0;
static const int16_t SERVO_COMMAND_MAX = 180;
static const int16_t SERVO_COMMAND_DEFAULT = 90;

constexpr size_t BUTTON_COUNT = ARRAY_LENGTH(DIGITAL_INPUTS);
constexpr size_t SERVO_COUNT = ARRAY_LENGTH(SERVO_CHANNELS);
constexpr size_t SERVO_COMMAND_BUFFER_LENGTH = SERVO_COUNT * 2;  // (servo_pin, command) pairs
constexpr size_t EXECUTOR_HANDLE_COUNT = 2;  // py_leds + aggregated servo commands

enum AgentState {
  WAITING_AGENT = 0,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
};

AgentState agent_state = WAITING_AGENT;

CRGB leds[LED_COUNT];

DigitalChannelState button_channels[BUTTON_COUNT];
Servo servo_handles[SERVO_COUNT];
rcl_subscription_t servo_command_sub;
std_msgs__msg__Int16MultiArray servo_command_msg;
bool servo_command_sub_initialized = false;
int16_t servo_command_buffer[SERVO_COMMAND_BUFFER_LENGTH] = {0};

Adafruit_AMG88xx amg;
bool therm_sensor_ready = false;
float thermo_buffer[AMG88xx_PIXEL_ARRAY_SIZE];
std_msgs__msg__Float32MultiArray thermo_msg;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t button_event_pub;
rcl_publisher_t thermo_pub;

rcl_subscription_t py_leds_sub;
std_msgs__msg__Int16 py_leds_msg;

std_msgs__msg__Int16 button_event_msg;

bool support_initialized = false;
bool node_initialized = false;
bool executor_initialized = false;
bool button_event_pub_initialized = false;
bool thermo_pub_initialized = false;
bool entities_ready = false;
bool py_leds_sub_initialized = false;

unsigned long last_button_sample_ms = 0;
unsigned long last_thermo_publish_ms = 0;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

void init_digital_channels();
bool read_channel(uint8_t pin);
bool should_publish_event(const DigitalChannelState &channel, bool state);
int16_t encode_event_payload(const DigitalChannelState &channel, bool state);
void configure_thermo_message();
bool create_entities();
void destroy_entities();
void publish_initial_states();
void process_periodic_tasks(unsigned long now);
void process_digital_inputs(unsigned long now);
void publish_thermo_frame();
void init_led_strip();
void set_strip_color(uint8_t r, uint8_t g, uint8_t b);
void apply_py_led_color(int16_t color_id);
void py_leds_callback(const void * msgin);
void init_servo_channels();
void configure_servo_command_message();
int16_t clamp_servo_command(int32_t value);
uint16_t servo_command_to_pulse(int16_t command);
void handle_servo_command(size_t servo_index, int16_t command);
int8_t servo_index_from_pin(int16_t pin);
void servo_commands_callback(const void * msgin);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial1.begin(SERIAL_BAUDRATE);
  set_microros_transports();

  init_digital_channels();
  configure_thermo_message();
  configure_servo_command_message();
  init_led_strip();
  init_servo_channels();

  Wire.begin();
  therm_sensor_ready = amg.begin();

  button_event_msg.data = 0;
  const unsigned long now = millis();
  last_button_sample_ms = now;
  last_thermo_publish_ms = now;
  agent_state = WAITING_AGENT;
}

void loop() {
  const unsigned long now = millis();

  switch (agent_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS,
        agent_state = (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) ?
          AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;
    case AGENT_AVAILABLE:
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
        digitalWrite(LED_BUILTIN, HIGH);
        publish_initial_states();
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS,
        agent_state = (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK) ?
          AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (agent_state == AGENT_CONNECTED && entities_ready && executor_initialized) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_TIMEOUT_MS));
        process_periodic_tasks(now);
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      digitalWrite(LED_BUILTIN, LOW);
      agent_state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (agent_state != AGENT_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(EXECUTOR_TIMEOUT_MS);
}

void init_digital_channels() {
  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    const DigitalInputConfig &config = DIGITAL_INPUTS[i];
    const uint8_t pin = config.pin;
    pinMode(pin, INPUT_PULLUP);
    button_channels[i].pin = pin;
    button_channels[i].kind = config.kind;
    button_channels[i].last_state = read_channel(pin);
    button_channels[i].last_change_ms = now;
  }

}

bool read_channel(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

bool should_publish_event(const DigitalChannelState &channel, bool state) {
  if (channel.kind == LATCHED_INPUT) {
    return true;
  }
  return state;
}

int16_t encode_event_payload(const DigitalChannelState &channel, bool state) {
  const int16_t pin = static_cast<int16_t>(channel.pin);
  if (channel.kind == LATCHED_INPUT) {
    return state ? pin : static_cast<int16_t>(-pin);
  }
  return state ? pin : 0;
}

void configure_thermo_message() {
  thermo_msg.layout.dim.capacity = 0;
  thermo_msg.layout.dim.size = 0;
  thermo_msg.layout.dim.data = nullptr;
  thermo_msg.layout.data_offset = 0;

  thermo_msg.data.data = thermo_buffer;
  thermo_msg.data.capacity = AMG88xx_PIXEL_ARRAY_SIZE;
  thermo_msg.data.size = AMG88xx_PIXEL_ARRAY_SIZE;

  for (size_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; ++i) {
    thermo_buffer[i] = 0.0f;
  }
}

void configure_servo_command_message() {
  servo_command_msg.layout.dim.capacity = 0;
  servo_command_msg.layout.dim.size = 0;
  servo_command_msg.layout.dim.data = nullptr;
  servo_command_msg.layout.data_offset = 0;

  for (size_t i = 0; i < SERVO_COMMAND_BUFFER_LENGTH; ++i) {
    servo_command_buffer[i] = 0;
  }

  servo_command_msg.data.data = servo_command_buffer;
  servo_command_msg.data.capacity = SERVO_COMMAND_BUFFER_LENGTH;
  servo_command_msg.data.size = 0;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
  support_initialized = true;

  RCCHECK(rclc_node_init_default(&node, "turtlebro_py_board", "", &support));
  node_initialized = true;

  RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLE_COUNT, &allocator));
  executor_initialized = true;

  RCCHECK(rclc_publisher_init_default(
    &button_event_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    BUTTON_TOPIC));
  button_event_pub_initialized = true;

  RCCHECK(rclc_publisher_init_default(
    &thermo_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    THERMO_TOPIC));
  thermo_pub_initialized = true;

  RCCHECK(rclc_subscription_init_default(
    &py_leds_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    PY_LEDS_TOPIC));
  py_leds_sub_initialized = true;
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &py_leds_sub,
    &py_leds_msg,
    py_leds_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_subscription_init_default(
    &servo_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    SERVO_COMMAND_TOPIC));
  servo_command_sub_initialized = true;
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &servo_command_sub,
    &servo_command_msg,
    servo_commands_callback,
    ON_NEW_DATA));

  entities_ready = true;
  return true;
}

void destroy_entities() {
  if (executor_initialized) {
    RCSOFTCHECK(rclc_executor_fini(&executor));
    executor_initialized = false;
  }

  if (py_leds_sub_initialized) {
    RCSOFTCHECK(rcl_subscription_fini(&py_leds_sub, &node));
    py_leds_sub_initialized = false;
  }

  if (servo_command_sub_initialized) {
    RCSOFTCHECK(rcl_subscription_fini(&servo_command_sub, &node));
    servo_command_sub_initialized = false;
  }

  if (thermo_pub_initialized) {
    RCSOFTCHECK(rcl_publisher_fini(&thermo_pub, &node));
    thermo_pub_initialized = false;
  }

  if (button_event_pub_initialized) {
    RCSOFTCHECK(rcl_publisher_fini(&button_event_pub, &node));
    button_event_pub_initialized = false;
  }

  if (node_initialized) {
    RCSOFTCHECK(rcl_node_fini(&node));
    node_initialized = false;
  }

  if (support_initialized) {
    RCSOFTCHECK(rclc_support_fini(&support));
    support_initialized = false;
  }

  entities_ready = false;
}

void publish_initial_states() {
  if (!button_event_pub_initialized) {
    return;
  }

  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    DigitalChannelState &channel = button_channels[i];
    const bool state = read_channel(channel.pin);
    channel.last_state = state;
    channel.last_change_ms = now;

    if (channel.kind != LATCHED_INPUT) {
      continue;
    }

    const int16_t payload = encode_event_payload(channel, state);
    if (payload == 0) {
      continue;
    }

    button_event_msg.data = payload;
    rcl_publish(&button_event_pub, &button_event_msg, nullptr);
  }
}

void process_periodic_tasks(unsigned long now) {
  if (!entities_ready) {
    return;
  }

  if ((now - last_button_sample_ms) >= BUTTON_SAMPLE_PERIOD_MS) {
    last_button_sample_ms = now;
    process_digital_inputs(now);
  }

  if (therm_sensor_ready && (now - last_thermo_publish_ms) >= THERMAL_SAMPLE_PERIOD_MS) {
    last_thermo_publish_ms = now;
    publish_thermo_frame();
  }
}

void process_digital_inputs(unsigned long now) {
  int16_t event_pin = 0;

  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    DigitalChannelState &channel = button_channels[i];
    bool state = read_channel(channel.pin);
    if (state != channel.last_state && (now - channel.last_change_ms) >= INPUT_DEBOUNCE_MS) {
      channel.last_state = state;
      channel.last_change_ms = now;

      if (should_publish_event(channel, state)) {
        event_pin = encode_event_payload(channel, state);
      }
    }
  }

  if (event_pin != 0) {
    button_event_msg.data = event_pin;
    rcl_publish(&button_event_pub, &button_event_msg, nullptr);
  }
}

void publish_thermo_frame() {
  if (!therm_sensor_ready || !thermo_pub_initialized) {
    return;
  }

  amg.readPixels(pixels);
  for (size_t i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; ++i) {
    thermo_buffer[i] = pixels[i];
  }

  rcl_publish(&thermo_pub, &thermo_msg, nullptr);
}

void init_led_strip() {
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, LED_COUNT);
  FastLED.setBrightness(LED_BRIGHTNESS);
  set_strip_color(0, 0, 0);
}

void set_strip_color(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < LED_COUNT; ++i) {
    leds[i].r = r;
    leds[i].g = g;
    leds[i].b = b;
  }
  FastLED.show();
}

void apply_py_led_color(int16_t color_id) {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  switch (color_id) {
    case 1:
      r = 255;
      break;
    case 2:
      g = 255;
      break;
    case 3:
      b = 255;
      break;
    case 4:
      r = 255;
      g = 255;
      break;
    case 5:
      r = 255;
      g = 255;
      b = 255;
      break;
    case 6:
    default:
      r = 0;
      g = 0;
      b = 0;
      break;
  }

  set_strip_color(r, g, b);
}

void py_leds_callback(const void * msgin) {
  if (msgin == nullptr) {
    return;
  }
  const std_msgs__msg__Int16 * msg = static_cast<const std_msgs__msg__Int16 *>(msgin);
  apply_py_led_color(msg->data);
}

void init_servo_channels() {
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    const ServoChannelConfig &config = SERVO_CHANNELS[i];
    servo_handles[i].attach(config.pin);
    const uint16_t pulse = servo_command_to_pulse(SERVO_COMMAND_DEFAULT);
    servo_handles[i].writeMicroseconds(pulse);
  }
}

int16_t clamp_servo_command(int32_t value) {
  if (value < SERVO_COMMAND_MIN) {
    return SERVO_COMMAND_MIN;
  }
  if (value > SERVO_COMMAND_MAX) {
    return SERVO_COMMAND_MAX;
  }
  return static_cast<int16_t>(value);
}

uint16_t servo_command_to_pulse(int16_t command) {
  const int16_t range = SERVO_COMMAND_MAX - SERVO_COMMAND_MIN;
  if (range <= 0) {
    return SERVO_CENTER_PULSE_US;
  }

  const int32_t normalized =
    static_cast<int32_t>(command - SERVO_COMMAND_MIN);
  const uint32_t pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
  const uint32_t pulse_offset =
    (pulse_range * static_cast<uint32_t>(normalized)) / static_cast<uint32_t>(range);
  return static_cast<uint16_t>(SERVO_MIN_PULSE_US + pulse_offset);
}

void handle_servo_command(size_t servo_index, int16_t command) {
  if (servo_index >= SERVO_COUNT) {
    return;
  }

  const int16_t clamped = clamp_servo_command(command);
  const uint16_t pulse = servo_command_to_pulse(clamped);
  servo_handles[servo_index].writeMicroseconds(pulse);
}

int8_t servo_index_from_pin(int16_t pin) {
  for (size_t i = 0; i < SERVO_COUNT; ++i) {
    if (static_cast<int16_t>(SERVO_CHANNELS[i].pin) == pin) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

void servo_commands_callback(const void * msgin) {
  if (msgin == nullptr) {
    return;
  }

  const std_msgs__msg__Int16MultiArray * msg =
    static_cast<const std_msgs__msg__Int16MultiArray *>(msgin);
  const size_t data_size = msg->data.size;
  if (data_size < 2) {
    return;
  }

  const size_t usable_size = data_size - (data_size % 2);
  for (size_t i = 0; i < usable_size; i += 2) {
    const int16_t servo_pin = msg->data.data[i];
    const int16_t command = msg->data.data[i + 1];
    const int8_t servo_index = servo_index_from_pin(servo_pin);
    if (servo_index < 0) {
      continue;
    }
    handle_servo_command(static_cast<size_t>(servo_index), command);
  }
}
