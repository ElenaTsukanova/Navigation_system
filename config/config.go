package config

import (
	"os"
	"time"

	"gopkg.in/yaml.v3"
)

type Config struct {
	EKF struct {
		TimeStep        float64 `yaml:"time_step"`
		StateSize       int     `yaml:"state_size"`
		MeasurementSize int     `yaml:"measurement_size"`
		InitialState    struct {
			Position   []float64 `yaml:"position"`
			Velocity   []float64 `yaml:"velocity"`
			Quaternion []float64 `yaml:"quaternion"`
			Bias_acc   []float64 `yaml:"bias_acc"`
			Bias_gyro  []float64 `yaml:"bias_gyro"`
		} `yaml:"initial_state"`
		InitialCov struct {
			Position   []float64 `yaml:"position"`
			Velocity   []float64 `yaml:"velocity"`
			Quaternion []float64 `yaml:"quaternion"`
			Bias_acc   []float64 `yaml:"bias_acc"`
			Bias_gyro  []float64 `yaml:"bias_gyro"`
		} `yaml:"initial_covariance"`
		ProcessNoise struct {
			Position   []float64 `yaml:"position"`
			Velocity   float64   `yaml:"velocity"`
			Quaternion []float64 `yaml:"quaternion"`
			Bias_acc   float64   `yaml:"bias_acc"`
			Bias_gyro  float64   `yaml:"bias_gyro"`
		} `yaml:"process_noise"`
		MeasurementNoise struct {
			Position_GNSS []float64 `yaml:"position_gnss"`
			Speed         float64   `yaml:"speed"`
		} `yaml:"measurement_noise"`
	} `yaml:"ekf"`

	Sensors struct {
		SyncThreshold time.Duration `yaml:"sync_threshold"`
		Accelerometer struct {
			Frequency float64 `yaml:"frequency"` // Частота акселерометра (Гц)
		} `yaml:"accelerometer"`
		Gyroscope struct {
			Frequency float64 `yaml:"frequency"` // Частота гироскопа (Гц)
		} `yaml:"gyroscope"`
		GNSS struct {
			Frequency          float64       `yaml:"frequency"`           // Частота GNSS (Гц)
			SyncWindow         time.Duration `yaml:"sync_window"`         // Окно синхронизации для GNSS
			ReferenceLatitude  float64       `yaml:"reference_latitude"`  // Широта (градусы)
			ReferenceLongitude float64       `yaml:"reference_longitude"` // Долгота (градусы)
			ReferenceAltitude  float64       `yaml:"reference_altitude"`  // Высота (метры)
		} `yaml:"gnss"`
	} `yaml:"sensors"`
}

func LoadConfig(filename string) (*Config, error) {
	data, err := os.ReadFile(filename)
	if err != nil {
		return nil, err
	}

	var cfg Config
	err = yaml.Unmarshal(data, &cfg)
	return &cfg, err
}
