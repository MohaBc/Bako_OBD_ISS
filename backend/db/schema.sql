-- =============================================================================
-- Bako OBD-ISS  |  MariaDB Schema v2.0.0
-- Apply:  mariadb -u $DB_USER -p$DB_PASS $DB_NAME < schema.sql
-- =============================================================================

SET NAMES utf8mb4;
SET time_zone          = '+00:00';
SET FOREIGN_KEY_CHECKS = 0;

-- =============================================================================
-- 1. vehicles
-- =============================================================================
CREATE TABLE IF NOT EXISTS vehicles (
    vehicle_id    INT UNSIGNED  NOT NULL AUTO_INCREMENT,
    serial_number VARCHAR(64)   NOT NULL UNIQUE  COMMENT 'Physical chassis serial number',
    vehicle_type  ENUM('BEE','BEE Van') NOT NULL,
    name          VARCHAR(128)  NOT NULL,
    active        TINYINT(1)    NOT NULL DEFAULT 1,
    created_at    DATETIME      NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (vehicle_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- =============================================================================
-- 2. vehicle_config  — pack geometry & calibration constants
-- =============================================================================
CREATE TABLE IF NOT EXISTS vehicle_config (
    vehicle_id        INT UNSIGNED      NOT NULL,
    cell_count        TINYINT UNSIGNED  NOT NULL DEFAULT 19,
    chemistry         VARCHAR(32)       NOT NULL DEFAULT 'LiFePO4',
    cell_min_mv       SMALLINT UNSIGNED NOT NULL DEFAULT 2500,
    cell_max_mv       SMALLINT UNSIGNED NOT NULL DEFAULT 3700,
    soc_empty_mv      SMALLINT UNSIGNED NOT NULL DEFAULT 2500  COMMENT '0 % SOC reference',
    soc_full_mv       SMALLINT UNSIGNED NOT NULL DEFAULT 3387  COMMENT '100 % SOC reference',
    pack_nominal_v    DECIMAL(6,2)      NOT NULL DEFAULT 60.80,
    pack_capacity_ah  DECIMAL(7,2)      NOT NULL DEFAULT 50.00,
    updated_at        DATETIME          NOT NULL DEFAULT CURRENT_TIMESTAMP
                                        ON UPDATE CURRENT_TIMESTAMP,
    PRIMARY KEY (vehicle_id),
    CONSTRAINT fk_cfg_vehicle FOREIGN KEY (vehicle_id)
        REFERENCES vehicles(vehicle_id) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- =============================================================================
-- 3. telemetry  — one row per frame, fully typed columns
-- =============================================================================
CREATE TABLE IF NOT EXISTS telemetry (
    frame_id         BIGINT UNSIGNED    NOT NULL AUTO_INCREMENT,
    vehicle_id       INT UNSIGNED       NOT NULL,
    device_ts        DATETIME(3)        NOT NULL  COMMENT 'Timestamp from device (UTC)',
    server_ts        DATETIME(3)        NOT NULL  DEFAULT CURRENT_TIMESTAMP(3),
    schema_version   VARCHAR(16)        NOT NULL  DEFAULT '2.0.0',
    transport        ENUM('http','tcp') NOT NULL  DEFAULT 'tcp',
    seq              INT UNSIGNED       NULL,

    -- battery pack
    pack_voltage_v   DECIMAL(6,3)       NULL,
    pack_current_a   DECIMAL(7,3)       NULL  COMMENT 'Positive = discharge',
    soc_pct          DECIMAL(5,2)       NULL  CHECK (soc_pct BETWEEN 0 AND 100),
    soc_bms_pct      DECIMAL(5,2)       NULL  CHECK (soc_bms_pct BETWEEN 0 AND 100),
    cell_count       TINYINT UNSIGNED   NULL,
    cell_avg_mv      DECIMAL(7,2)       NULL,
    cell_min_mv      DECIMAL(7,2)       NULL,
    cell_max_mv      DECIMAL(7,2)       NULL,
    cell_spread_mv   DECIMAL(7,2)       NULL,
    fault_level      TINYINT UNSIGNED   NULL  CHECK (fault_level IN (0,1,2)),
    error_code       SMALLINT UNSIGNED  NULL,
    flag_ready       TINYINT(1)         NULL,
    flag_charging    TINYINT(1)         NULL,
    flag_discharging TINYINT(1)         NULL,

    -- charge limit (battery.charge_limit)
    charge_limit_v   DECIMAL(6,3)       NULL  COMMENT 'max_charge_v (V)',
    charge_limit_a   DECIMAL(6,2)       NULL  COMMENT 'max_charge_a (A)',
    charge_enable    TINYINT(1)         NULL,

    -- temperatures (°C)
    battery_avg_c    DECIMAL(5,2)       NULL,
    battery_min_c    DECIMAL(5,2)       NULL,
    battery_max_c    DECIMAL(5,2)       NULL,
    dcdc_temp_c      DECIMAL(5,2)       NULL,
    motor_temp_c     DECIMAL(5,2)       NULL,
    mppt_temp_c      DECIMAL(5,2)       NULL,
    battery_temps_c  JSON               NULL  COMMENT 'Array of 4 probe temps in °C',

    -- solar
    solar_in_v       DECIMAL(6,3)       NULL  COMMENT 'Pre-MPPT panel voltage (V)',
    solar_in_a       DECIMAL(6,3)       NULL  COMMENT 'Pre-MPPT panel current (A)',
    solar_out_a      DECIMAL(6,3)       NULL  COMMENT 'Post-MPPT current (A)',

    -- DC/DC
    dcdc_64v         DECIMAL(6,3)       NULL,
    dcdc_12v         DECIMAL(6,3)       NULL,

    -- GNSS
    latitude         DECIMAL(10,7)      NULL  CHECK (latitude  BETWEEN -90  AND  90),
    longitude        DECIMAL(10,7)      NULL  CHECK (longitude BETWEEN -180 AND 180),
    speed_kmh        DECIMAL(6,2)       NULL,
    gnss_fix         TINYINT(1)         NULL,

    -- vehicle
    handbrake        TINYINT(1)         NULL,

    -- cell voltages — plain array of 19 integers in mV
    cells_voltages   JSON               NULL  COMMENT 'Array of 19 integers (mV), index 0 = cell 1',

    PRIMARY KEY (frame_id),
    INDEX idx_tel_vehicle_ts (vehicle_id, device_ts),
    INDEX idx_tel_server_ts  (server_ts),
    CONSTRAINT fk_tel_vehicle FOREIGN KEY (vehicle_id)
        REFERENCES vehicles(vehicle_id) ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- =============================================================================
-- SEED DATA
-- =============================================================================

SET FOREIGN_KEY_CHECKS = 1;

INSERT INTO vehicles (serial_number, vehicle_type, name, active) VALUES
    ('BEE-2024-001', 'BEE',     'BEE Prototype #1', 1),
    ('BEE-2024-002', 'BEE Van', 'BEE Van #1',       1)
ON DUPLICATE KEY UPDATE name = VALUES(name);

-- vehicle_id 1 = BEE, vehicle_id 2 = BEE Van (auto-assigned above)
INSERT INTO vehicle_config (vehicle_id, cell_count, chemistry, cell_min_mv, cell_max_mv,
                             soc_empty_mv, soc_full_mv, pack_nominal_v, pack_capacity_ah)
VALUES
    (1, 19, 'LiFePO4', 2500, 3700, 2500, 3387, 60.80, 50.00),
    (2, 19, 'LiFePO4', 2500, 3700, 2500, 3387, 60.80, 50.00)
ON DUPLICATE KEY UPDATE cell_count = VALUES(cell_count);
