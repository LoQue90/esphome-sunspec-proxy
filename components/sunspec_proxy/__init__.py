import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    UNIT_WATT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_KILOWATT_HOURS,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_CONNECTIVITY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = []
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

# Component configuration keys
CONF_TCP_PORT = "tcp_port"
CONF_UNIT_ID = "unit_id"
CONF_DTU_HOST = "dtu_host"         # DTU-Pro IP address or hostname
CONF_DTU_PORT = "dtu_port"         # DTU-Pro Modbus TCP port (default 502)
CONF_DTU_ADDRESS = "dtu_address"   # DTU Modbus unit ID (default 101)
CONF_PHASES = "phases"
CONF_RATED_VOLTAGE_V = "rated_voltage_v"
CONF_MANUFACTURER = "manufacturer"
CONF_MODEL_NAME = "model_name"
CONF_SERIAL_NUMBER = "serial_number"
CONF_POLL_INTERVAL_MS = "poll_interval_ms"
CONF_TCP_TIMEOUT_MS = "tcp_timeout_ms"
CONF_RTU_SOURCES = "rtu_sources"  # Keep name for backward compat, but now represents inverters

# Per-source configuration keys
CONF_RTU_ADDRESS = "rtu_address"  # Legacy: treat as port number if < 100
CONF_PORT = "port"  # DTU port number (0, 1, 2... for each connected inverter)
CONF_RATED_POWER_W = "rated_power_w"
CONF_CONNECTED_PHASE = "connected_phase"
CONF_INVERTER_MODEL = "inverter_model"
CONF_INVERTER_SERIAL = "inverter_serial"
CONF_MPPT_INPUTS = "mppt_inputs"

# Sensor enable flags
CONF_SENSORS = "sensors"
CONF_MPPT_SENSORS = "mppt_sensors"  # Enable per-MPPT sensors
CONF_SENSOR_POWER = "power"
CONF_SENSOR_VOLTAGE = "voltage"
CONF_SENSOR_CURRENT = "current"
CONF_SENSOR_FREQUENCY = "frequency"
CONF_SENSOR_ENERGY = "energy"
CONF_SENSOR_TEMPERATURE = "temperature"
CONF_SENSOR_PV_VOLTAGE = "pv_voltage"  # DC voltage (per MPPT when enabled)
CONF_SENSOR_PV_CURRENT = "pv_current"  # DC current (per MPPT when enabled)
CONF_SENSOR_PV_POWER = "pv_power"      # DC power (per MPPT when enabled)
CONF_SENSOR_ALARM_CODE = "alarm_code"
CONF_SENSOR_ALARM_COUNT = "alarm_count"
CONF_SENSOR_LINK_STATUS = "link_status"
CONF_SENSOR_OPERATING_STATUS = "operating_status"
CONF_SENSOR_TODAY_ENERGY = "today_energy"

# Aggregate sensor enable flags
CONF_AGGREGATE_SENSORS = "aggregate_sensors"

# Bridge status sensor enable flags
CONF_BRIDGE_SENSORS = "bridge_sensors"
CONF_SENSOR_TCP_CLIENTS = "tcp_clients"
CONF_SENSOR_TCP_REQUESTS = "tcp_requests"
CONF_SENSOR_TCP_ERRORS = "tcp_errors"
CONF_SENSOR_VICTRON_CONNECTED = "victron_connected"
CONF_SENSOR_POWER_LIMIT = "power_limit"
CONF_SENSOR_DTU_SERIAL = "dtu_serial"
CONF_SENSOR_DTU_ONLINE = "dtu_online"
CONF_SENSOR_DTU_POLL_SUCCESS = "dtu_poll_success"
CONF_SENSOR_DTU_POLL_FAIL = "dtu_poll_fail"

sunspec_proxy_ns = cg.esphome_ns.namespace("sunspec_proxy")
SunSpecProxy = sunspec_proxy_ns.class_("SunSpecProxy", cg.Component)

# Known Hoymiles inverter models with their characteristics
HOYMILES_MODELS = {
    # HM Single-phase microinverters - 1 MPPT
    "HM-300": {"phases": 1, "mppt": 1, "power": 300},
    "HM-350": {"phases": 1, "mppt": 1, "power": 350},
    "HM-400": {"phases": 1, "mppt": 1, "power": 400}, 
    # HM Single-phase microinverters - 2 MPPT
    "HM-600": {"phases": 1, "mppt": 2, "power": 600},
    "HM-700": {"phases": 1, "mppt": 2, "power": 700},
    "HM-800": {"phases": 1, "mppt": 2, "power": 800},
    # HM Single-phase microinverters - 4 MPPT
    "HM-1200": {"phases": 1, "mppt": 4, "power": 1200},
    "HM-1500": {"phases": 1, "mppt": 4, "power": 1500},
    # Single-phase microinverters (HMS series) - 1 MPPT
    "HMS-300-1T": {"phases": 1, "mppt": 1, "power": 300},
    "HMS-350-1T": {"phases": 1, "mppt": 1, "power": 350},
    "HMS-400-1T": {"phases": 1, "mppt": 1, "power": 400},
    "HMS-450-1T": {"phases": 1, "mppt": 1, "power": 450},
    "HMS-500-1T": {"phases": 1, "mppt": 1, "power": 500},
    # Single-phase microinverters (HMS series) - 2 MPPT
    "HMS-600-2T": {"phases": 1, "mppt": 2, "power": 600},
    "HMS-700-2T": {"phases": 1, "mppt": 2, "power": 700},
    "HMS-800-2T": {"phases": 1, "mppt": 2, "power": 800},
    "HMS-900-2T": {"phases": 1, "mppt": 2, "power": 900},
    "HMS-1000-2T": {"phases": 1, "mppt": 2, "power": 1000},
    # Single-phase microinverters (HMS series) - 4 MPPT
    "HMS-1600-4T": {"phases": 1, "mppt": 4, "power": 1600},
    "HMS-1800-4T": {"phases": 1, "mppt": 4, "power": 1800},
    "HMS-2000-4T": {"phases": 1, "mppt": 4, "power": 2000},
    # Three-phase microinverters (HMT series) - 4 MPPT
    "HMT-1600-4T": {"phases": 3, "mppt": 4, "power": 1600},
    "HMT-1800-4T": {"phases": 3, "mppt": 4, "power": 1800},
    "HMT-2000-4T": {"phases": 3, "mppt": 4, "power": 2000},
    # Three-phase microinverters (HMT series) - 6 MPPT
    "HMT-2250-6T": {"phases": 3, "mppt": 6, "power": 2250},
    # MIT series
    "MIT-1300-2T": {"phases": 3, "mppt": 2, "power": 1300},
    "MIT-1500-2T": {"phases": 3, "mppt": 2, "power": 1500},
    "MIT-1600-4T": {"phases": 3, "mppt": 4, "power": 1600},
    "MIT-1800-4T": {"phases": 3, "mppt": 4, "power": 1800},
    "MIT-2000-4T": {"phases": 3, "mppt": 4, "power": 2000},
    "MIT-2500-6T": {"phases": 3, "mppt": 6, "power": 2500},
    "MIT-3000-6T": {"phases": 3, "mppt": 6, "power": 3000},
    "MIT-4000-8T": {"phases": 3, "mppt": 8, "power": 4000},
    "MIT-5000-8T": {"phases": 3, "mppt": 8, "power": 5000},
    "MIT-6000-8T": {"phases": 3, "mppt": 8, "power": 6000},
    # Generic fallback
    "GENERIC-1P": {"phases": 1, "mppt": 2, "power": 1000},
    "GENERIC-3P": {"phases": 3, "mppt": 4, "power": 3000},
}

# Sensor configuration schemas
SOURCE_SENSORS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_SENSOR_POWER, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_VOLTAGE, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_CURRENT, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_FREQUENCY, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_ENERGY, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_TEMPERATURE, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_PV_VOLTAGE, default=False): cv.boolean,
        cv.Optional(CONF_SENSOR_PV_CURRENT, default=False): cv.boolean,
        cv.Optional(CONF_SENSOR_PV_POWER, default=False): cv.boolean,
        cv.Optional(CONF_SENSOR_ALARM_CODE, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_ALARM_COUNT, default=False): cv.boolean,
        cv.Optional(CONF_SENSOR_LINK_STATUS, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_OPERATING_STATUS, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_TODAY_ENERGY, default=True): cv.boolean,
        cv.Optional(CONF_MPPT_SENSORS, default=False): cv.boolean,  # Enable per-MPPT sensors
    }
)

AGGREGATE_SENSORS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_SENSOR_POWER, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_VOLTAGE, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_CURRENT, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_FREQUENCY, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_ENERGY, default=True): cv.boolean,
    }
)

BRIDGE_SENSORS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_SENSOR_TCP_CLIENTS, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_TCP_REQUESTS, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_TCP_ERRORS, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_VICTRON_CONNECTED, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_POWER_LIMIT, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_DTU_SERIAL, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_DTU_ONLINE, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_DTU_POLL_SUCCESS, default=True): cv.boolean,
        cv.Optional(CONF_SENSOR_DTU_POLL_FAIL, default=True): cv.boolean,
    }
)

# RTU source (inverter) configuration schema
# Note: rtu_address is now interpreted as port number (0, 1, 2...) if < 100
# For backwards compatibility, values >= 100 are treated as legacy behavior
RTU_SOURCE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_RTU_ADDRESS): cv.int_range(min=0, max=255),  # Legacy or port number
        cv.Optional(CONF_PORT): cv.int_range(min=0, max=15),  # DTU port number (preferred)
        cv.Required(CONF_INVERTER_MODEL): cv.one_of(*HOYMILES_MODELS.keys(), upper=True),
        cv.Optional(CONF_INVERTER_SERIAL, default=""): cv.string,
        cv.Optional(CONF_NAME, default=""): cv.string,
        cv.Optional(CONF_CONNECTED_PHASE, default=1): cv.int_range(min=1, max=3),
        cv.Optional(CONF_PHASES): cv.int_range(min=1, max=3),
        cv.Optional(CONF_RATED_POWER_W): cv.int_range(min=1),
        cv.Optional(CONF_MPPT_INPUTS): cv.int_range(min=1, max=8),
        cv.Optional(CONF_SENSORS, default={}): SOURCE_SENSORS_SCHEMA,
    }
)

# Main component configuration schema
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SunSpecProxy),
        cv.Required(CONF_DTU_HOST): cv.string,
        cv.Optional(CONF_DTU_PORT, default=502): cv.port,
        cv.Optional(CONF_DTU_ADDRESS, default=101): cv.int_range(min=1, max=254),
        cv.Optional(CONF_TCP_PORT, default=502): cv.port,
        cv.Optional(CONF_UNIT_ID, default=126): cv.int_range(min=1, max=247),
        cv.Optional(CONF_PHASES, default=3): cv.int_range(min=1, max=3),
        cv.Optional(CONF_RATED_VOLTAGE_V, default=230): cv.int_range(min=1),
        cv.Optional(CONF_MANUFACTURER, default="Fronius"): cv.string,
        cv.Optional(CONF_MODEL_NAME, default="Hoymiles Bridge"): cv.string,
        cv.Optional(CONF_SERIAL_NUMBER, default="HM-BRIDGE-001"): cv.string,
        cv.Required(CONF_RTU_SOURCES): cv.All(
            cv.ensure_list(RTU_SOURCE_SCHEMA), cv.Length(min=1, max=8)
        ),
        cv.Optional(CONF_POLL_INTERVAL_MS, default=5000): cv.int_range(min=1000),
        cv.Optional(CONF_TCP_TIMEOUT_MS, default=3000): cv.int_range(min=100),
        cv.Optional(CONF_AGGREGATE_SENSORS, default={}): AGGREGATE_SENSORS_SCHEMA,
        cv.Optional(CONF_BRIDGE_SENSORS, default={}): BRIDGE_SENSORS_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


def get_model_specs(model_name):
    return HOYMILES_MODELS.get(model_name, HOYMILES_MODELS["GENERIC-1P"])


# Counter for generating unique IDs
_sensor_counter = 0

def _make_sensor_id():
    global _sensor_counter
    _sensor_counter += 1
    return f"sunspec_sensor_{_sensor_counter}"


async def _create_sensor(name, unit=None, accuracy=0, device_class=None, state_class=None, icon=None, entity_category=None):
    """Create a sensor with the given parameters."""
    sens_id = cv.declare_id(sensor.Sensor)(_make_sensor_id())
    
    # Build schema kwargs for defaults
    schema_kwargs = {}
    if unit:
        schema_kwargs["unit_of_measurement"] = unit
    if accuracy is not None:
        schema_kwargs["accuracy_decimals"] = accuracy
    if device_class:
        schema_kwargs["device_class"] = device_class
    if state_class:
        schema_kwargs["state_class"] = state_class
    if icon:
        schema_kwargs["icon"] = icon
    if entity_category:
        schema_kwargs["entity_category"] = entity_category
    
    # Config dict with id and name (required by schema)
    conf = {
        CONF_ID: sens_id,
        CONF_NAME: name,
    }
    
    # Create schema with defaults, then validate config through it
    schema = sensor.sensor_schema(**schema_kwargs)
    validated = schema(conf)
    return await sensor.new_sensor(validated)


async def _create_binary_sensor(name, device_class=None):
    """Create a binary sensor."""
    sens_id = cv.declare_id(binary_sensor.BinarySensor)(_make_sensor_id())
    
    schema_kwargs = {}
    if device_class:
        schema_kwargs["device_class"] = device_class
    
    conf = {
        CONF_ID: sens_id,
        CONF_NAME: name,
    }
    
    schema = binary_sensor.binary_sensor_schema(**schema_kwargs)
    validated = schema(conf)
    return await binary_sensor.new_binary_sensor(validated)


async def _create_text_sensor(name, icon=None, entity_category=None):
    """Create a text sensor."""
    sens_id = cv.declare_id(text_sensor.TextSensor)(_make_sensor_id())
    
    schema_kwargs = {}
    if icon:
        schema_kwargs["icon"] = icon
    if entity_category:
        schema_kwargs["entity_category"] = entity_category
    
    conf = {
        CONF_ID: sens_id,
        CONF_NAME: name,
    }
    
    schema = text_sensor.text_sensor_schema(**schema_kwargs)
    validated = schema(conf)
    return await text_sensor.new_text_sensor(validated)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_dtu_host(config[CONF_DTU_HOST]))
    cg.add(var.set_dtu_port(config[CONF_DTU_PORT]))
    cg.add(var.set_dtu_address(config[CONF_DTU_ADDRESS]))
    cg.add(var.set_tcp_port(config[CONF_TCP_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))
    cg.add(var.set_phases(config[CONF_PHASES]))
    cg.add(var.set_rated_voltage(config[CONF_RATED_VOLTAGE_V]))
    cg.add(var.set_manufacturer(config[CONF_MANUFACTURER]))
    cg.add(var.set_model_name(config[CONF_MODEL_NAME]))
    cg.add(var.set_serial_number(config[CONF_SERIAL_NUMBER]))
    cg.add(var.set_poll_interval_ms(config[CONF_POLL_INTERVAL_MS]))
    cg.add(var.set_tcp_timeout_ms(config[CONF_TCP_TIMEOUT_MS]))

    # Process RTU sources (inverter ports on the DTU)
    for idx, src in enumerate(config[CONF_RTU_SOURCES]):
        model_name = src[CONF_INVERTER_MODEL]
        model_specs = get_model_specs(model_name)

        phases = src.get(CONF_PHASES, model_specs["phases"])
        rated_power = src.get(CONF_RATED_POWER_W, model_specs["power"])
        mppt_inputs = src.get(CONF_MPPT_INPUTS, model_specs["mppt"])
        name = src.get(CONF_NAME) or model_name
        serial = src.get(CONF_INVERTER_SERIAL, "")
        
        # Determine port number: prefer explicit 'port', fall back to 'rtu_address'
        # If rtu_address >= 100, treat it as a legacy Modbus address (not supported anymore)
        # If rtu_address < 100 or port is specified, use as port number
        if CONF_PORT in src:
            port_num = src[CONF_PORT]
        elif CONF_RTU_ADDRESS in src:
            port_num = src[CONF_RTU_ADDRESS]
            # If the user specified something like 90/91, map to 0/1 for now
            if port_num >= 90 and port_num < 100:
                port_num = port_num - 90  # Convert 90->0, 91->1, etc.
        else:
            port_num = idx  # Default to index

        cg.add(
            var.add_rtu_source(
                port_num,  # Now this is the DTU port number, not a Modbus address
                phases,
                rated_power,
                src[CONF_CONNECTED_PHASE],
                mppt_inputs,
                name,
                model_name,
                serial,
            )
        )

        sensor_config = src.get(CONF_SENSORS, {})

        if sensor_config.get(CONF_SENSOR_POWER, True):
            sens = await _create_sensor(f"{name} Power", UNIT_WATT, 0, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_power_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_VOLTAGE, True):
            sens = await _create_sensor(f"{name} Voltage", UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_voltage_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_CURRENT, True):
            sens = await _create_sensor(f"{name} Current", UNIT_AMPERE, 2, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_current_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_FREQUENCY, True):
            sens = await _create_sensor(f"{name} Frequency", UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_frequency_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_ENERGY, True):
            sens = await _create_sensor(f"{name} Energy", UNIT_KILOWATT_HOURS, 1, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING)
            cg.add(var.set_source_energy_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_TODAY_ENERGY, True):
            sens = await _create_sensor(f"{name} Today Energy", "Wh", 0, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING)
            cg.add(var.set_source_today_energy_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_TEMPERATURE, True):
            sens = await _create_sensor(f"{name} Temperature", UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_temperature_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_PV_VOLTAGE, False):
            sens = await _create_sensor(f"{name} PV Voltage", UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_pv_voltage_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_PV_CURRENT, False):
            sens = await _create_sensor(f"{name} PV Current", UNIT_AMPERE, 2, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_pv_current_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_PV_POWER, False):
            sens = await _create_sensor(f"{name} PV Power", UNIT_WATT, 0, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT)
            cg.add(var.set_source_pv_power_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_ALARM_CODE, True):
            sens = await _create_sensor(f"{name} Alarm Code", None, 0, None, None, "mdi:alert-circle", ENTITY_CATEGORY_DIAGNOSTIC)
            cg.add(var.set_source_alarm_code_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_ALARM_COUNT, False):
            sens = await _create_sensor(f"{name} Alarm Count", None, 0, None, None, "mdi:alert-circle-outline", ENTITY_CATEGORY_DIAGNOSTIC)
            cg.add(var.set_source_alarm_count_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_LINK_STATUS, True):
            sens = await _create_sensor(f"{name} Link Status", None, 0, None, None, "mdi:signal", ENTITY_CATEGORY_DIAGNOSTIC)
            cg.add(var.set_source_link_status_sensor(idx, sens))

        if sensor_config.get(CONF_SENSOR_OPERATING_STATUS, True):
            tsens = await _create_text_sensor(f"{name} Status", "mdi:information-outline", ENTITY_CATEGORY_DIAGNOSTIC)
            cg.add(var.set_source_status_sensor(idx, tsens))

        # Always create online and poll stats sensors
        bsens = await _create_binary_sensor(f"{name} Online", DEVICE_CLASS_CONNECTIVITY)
        cg.add(var.set_source_online_sensor(idx, bsens))

        sens = await _create_sensor(f"{name} Poll Success", None, 0, None, None, "mdi:check-circle", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_source_poll_success_sensor(idx, sens))

        sens = await _create_sensor(f"{name} Poll Failures", None, 0, None, None, "mdi:alert-circle", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_source_poll_fail_sensor(idx, sens))

        # Per-MPPT sensors if enabled
        if sensor_config.get(CONF_MPPT_SENSORS, False):
            for mppt_idx in range(mppt_inputs):
                mppt_num = mppt_idx + 1
                mppt_name = f"{name} MPPT{mppt_num}"
                
                # DC voltage
                sens = await _create_sensor(f"{mppt_name} DC Voltage", UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_dc_voltage_sensor(idx, mppt_idx, sens))
                
                # DC current
                sens = await _create_sensor(f"{mppt_name} DC Current", UNIT_AMPERE, 2, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_dc_current_sensor(idx, mppt_idx, sens))
                
                # DC power
                sens = await _create_sensor(f"{mppt_name} DC Power", UNIT_WATT, 0, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_dc_power_sensor(idx, mppt_idx, sens))
                
                # AC voltage
                sens = await _create_sensor(f"{mppt_name} AC Voltage", UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_ac_voltage_sensor(idx, mppt_idx, sens))
                
                # Frequency
                sens = await _create_sensor(f"{mppt_name} Frequency", UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_frequency_sensor(idx, mppt_idx, sens))
                
                # Power
                sens = await _create_sensor(f"{mppt_name} Power", UNIT_WATT, 0, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_power_sensor(idx, mppt_idx, sens))
                
                # Today energy
                sens = await _create_sensor(f"{mppt_name} Today Energy", "Wh", 0, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING)
                cg.add(var.set_mppt_today_energy_sensor(idx, mppt_idx, sens))
                
                # Total energy
                sens = await _create_sensor(f"{mppt_name} Total Energy", UNIT_KILOWATT_HOURS, 1, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING)
                cg.add(var.set_mppt_total_energy_sensor(idx, mppt_idx, sens))
                
                # Temperature
                sens = await _create_sensor(f"{mppt_name} Temperature", UNIT_CELSIUS, 1, DEVICE_CLASS_TEMPERATURE, STATE_CLASS_MEASUREMENT)
                cg.add(var.set_mppt_temperature_sensor(idx, mppt_idx, sens))

    # Aggregate sensors
    agg_config = config.get(CONF_AGGREGATE_SENSORS, {})

    if agg_config.get(CONF_SENSOR_POWER, True):
        sens = await _create_sensor("Aggregate Power", UNIT_WATT, 0, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT, "mdi:solar-power")
        cg.add(var.set_agg_power_sensor(sens))

    if agg_config.get(CONF_SENSOR_VOLTAGE, True):
        sens = await _create_sensor("Aggregate Voltage", UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE, STATE_CLASS_MEASUREMENT)
        cg.add(var.set_agg_voltage_sensor(sens))

    if agg_config.get(CONF_SENSOR_CURRENT, True):
        sens = await _create_sensor("Aggregate Current", UNIT_AMPERE, 2, DEVICE_CLASS_CURRENT, STATE_CLASS_MEASUREMENT)
        cg.add(var.set_agg_current_sensor(sens))

    if agg_config.get(CONF_SENSOR_FREQUENCY, True):
        sens = await _create_sensor("Aggregate Frequency", UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY, STATE_CLASS_MEASUREMENT)
        cg.add(var.set_agg_frequency_sensor(sens))

    if agg_config.get(CONF_SENSOR_ENERGY, True):
        sens = await _create_sensor("Aggregate Energy", UNIT_KILOWATT_HOURS, 1, DEVICE_CLASS_ENERGY, STATE_CLASS_TOTAL_INCREASING)
        cg.add(var.set_agg_energy_sensor(sens))

    # Bridge status sensors
    bridge_config = config.get(CONF_BRIDGE_SENSORS, {})

    if bridge_config.get(CONF_SENSOR_TCP_CLIENTS, True):
        sens = await _create_sensor("TCP Clients", None, 0, None, None, "mdi:lan-connect", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_tcp_clients_sensor(sens))

    if bridge_config.get(CONF_SENSOR_TCP_REQUESTS, True):
        sens = await _create_sensor("TCP Requests Total", None, 0, None, None, "mdi:counter", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_tcp_requests_sensor(sens))

    if bridge_config.get(CONF_SENSOR_TCP_ERRORS, True):
        sens = await _create_sensor("TCP Errors Total", None, 0, None, None, "mdi:alert-octagon", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_tcp_errors_sensor(sens))

    if bridge_config.get(CONF_SENSOR_VICTRON_CONNECTED, True):
        bsens = await _create_binary_sensor("Victron Connected", DEVICE_CLASS_CONNECTIVITY)
        cg.add(var.set_victron_connected_sensor(bsens))

        tsens = await _create_text_sensor("Victron Status", "mdi:information-outline")
        cg.add(var.set_victron_status_sensor(tsens))

    if bridge_config.get(CONF_SENSOR_POWER_LIMIT, True):
        sens = await _create_sensor("Power Limit", UNIT_PERCENT, 1, None, None, "mdi:speedometer")
        cg.add(var.set_power_limit_sensor(sens))

    # DTU diagnostic sensors
    if bridge_config.get(CONF_SENSOR_DTU_SERIAL, True):
        tsens = await _create_text_sensor("DTU Serial", "mdi:identifier", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_dtu_serial_sensor(tsens))

    if bridge_config.get(CONF_SENSOR_DTU_ONLINE, True):
        bsens = await _create_binary_sensor("DTU Online", DEVICE_CLASS_CONNECTIVITY)
        cg.add(var.set_dtu_online_sensor(bsens))

    if bridge_config.get(CONF_SENSOR_DTU_POLL_SUCCESS, True):
        sens = await _create_sensor("DTU Poll Success", None, 0, None, None, "mdi:check-circle", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_dtu_poll_ok_sensor(sens))

    if bridge_config.get(CONF_SENSOR_DTU_POLL_FAIL, True):
        sens = await _create_sensor("DTU Poll Failures", None, 0, None, None, "mdi:alert-circle", ENTITY_CATEGORY_DIAGNOSTIC)
        cg.add(var.set_dtu_poll_fail_sensor(sens))
