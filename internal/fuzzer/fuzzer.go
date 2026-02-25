package fuzzer

import (
	"fmt"
	"math"

	"time" // для реального случая

	"gonum.org/v1/gonum/mat"
	"main.go/config"
	"main.go/internal/ekf"
	"main.go/internal/models"
)

// Fuzzer обрабатывает данные датчиков
type Fuzzer struct {
	cfg *config.Config
	ekf *ekf.EKFWrapper
	p   *models.PositionModel

	lastTimeGNSS time.Time //в реальности
	//lastTimeGNSS float64 // для отладки
	//nextTimeGNSS float64 // для отладки
	sizeBufBias  int
	BufBiasAccX  []float64
	BufBiasAccY  []float64
	BufBiasAccZ  []float64
	BufBiasGyroX []float64
	BufBiasGyroY []float64
	BufBiasGyroZ []float64

	gravity float64
}

// NewDataProcessor создает новый процессор
func NewFuzzer(cfg *config.Config) *Fuzzer {
	return &Fuzzer{
		cfg:     cfg,
		gravity: 9.81,

		lastTimeGNSS: time.Now(),
		//lastTimeGNSS: 0.0,
		//nextTimeGNSS: 0.0,
		sizeBufBias:  10,
		BufBiasAccX:  make([]float64, 10),
		BufBiasAccY:  make([]float64, 10),
		BufBiasAccZ:  make([]float64, 10),
		BufBiasGyroX: make([]float64, 10),
		BufBiasGyroY: make([]float64, 10),
		BufBiasGyroZ: make([]float64, 10),
	}
}

// Process обрабатывает данные датчиков
func (f *Fuzzer) Process(syncedData []models.SynchronizedData,
) ([]models.EstimatedState, error) {

	count_GNSS := 0
	initialization_on := false

	calc_bias_on := false

	// Обрабатываем синхронизированные данные
	var results []models.EstimatedState

	for i, data := range syncedData {
		var state *models.EstimatedState
		var err error

		data.AccelX *= -f.gravity
		data.AccelY *= -f.gravity
		data.AccelZ *= -f.gravity
		// DegreesToRadians преобразует градусы/с в радианы/c
		data.GyroX = DegreesToRadians(data.GyroX)
		data.GyroY = DegreesToRadians(data.GyroY)
		data.GyroZ = DegreesToRadians(data.GyroZ)

		if initialization_on == false {

			if data.HasGNSS {

				//f.nextTimeGNSS += 1 / f.cfg.Sensors.GNSS.Frequency // для отладки
				count_GNSS++
				if count_GNSS == 2 {
					initialization_on = true

					// ИНИЦИАЛИЗАЦИЯ

					// Инициализируем насальные параметры
					err = f.initState(data)
					// Инициализируем EKF
					if err := f.initEKF(); err != nil {
						return nil, err
					}

				} else {
					f.lastTimeGNSS = time.Now() // на реальном эксперименте
					//f.lastTimeGNSS = f.nextTimeGNSS // для отладки

					f.cfg.Sensors.GNSS.ReferenceLatitude = data.Latitude
					f.cfg.Sensors.GNSS.ReferenceLongitude = data.Longitude
					f.cfg.Sensors.GNSS.ReferenceAltitude = data.Altitude

					//E, N, U := GeodeticToENU(data.Latitude, data.Longitude, data.Altitude, f.cfg)
					//fmt.Print("\nE = ", E, "; N = ", N, "; U = ", U)

					//lat, lon, alt := ENUToGeodetic(0, 0, 0, f.cfg)
					//fmt.Print("\nLat = ", lat, "; Lon = ", lon, "; Alt = ", alt)

					calc_bias_on = true

				}
			} else {

				if calc_bias_on {
					f.calcAverageData(data)
				}

			}
		} else {

			if data.HasGNSS {
				// Есть GNSS данные - полный шаг EKF

				// если пришли данные IMU, преобразуем поворот осей Ox, Oy к направлению движения из 45* в 90* и записываем в новый влияющий вектор
				// у тебя эти преобразования будут только в каналах Acc и Gyro
				accXCorrect, accYCorrect, accZCorrect := IMUTransformation_2(data.AccelX, data.AccelY, data.AccelZ)
				gyroXCorrect, gyroYCorrect, gyroZCorrect := IMUTransformation_2(data.GyroX, data.GyroY, data.GyroZ)

				// Входной вектор
				u := mat.NewVecDense(6, []float64{
					accXCorrect, accYCorrect, accZCorrect,
					gyroXCorrect, gyroYCorrect, gyroZCorrect,
				})

				gnssX_ENU, gnssY_ENU, gnssZ_ENU := GeodeticToENU(data.Latitude, data.Longitude, data.Altitude, f.cfg)

				// Вектор измерений
				z := mat.NewVecDense(4, []float64{
					gnssX_ENU, gnssY_ENU, gnssZ_ENU,
					data.Speed,
				})

				state, err = f.ekf.Run(u, z)

			} else {
				// Только IMU данные - только предсказание

				// если пришли данные IMU, преобразуем поворот осей Ox, Oy к направлению движения из 45* в 90* и записываем в новый влияющий вектор
				accXCorrect, accYCorrect, accZCorrect := IMUTransformation_2(data.AccelX, data.AccelY, data.AccelZ)
				gyroXCorrect, gyroYCorrect, gyroZCorrect := IMUTransformation_2(data.GyroX, data.GyroY, data.GyroZ)

				// Входной вектор
				u := mat.NewVecDense(6, []float64{
					accXCorrect, accYCorrect, accZCorrect,
					gyroXCorrect, gyroYCorrect, gyroZCorrect,
				})

				// если канал acc - копируем в вектор gyro предыдущие значения
				// если канал gyro - копируем в вектор acc предыдущие значения

				state, err = f.ekf.Predict(u)

				fmt.Printf("\nX_ENU: %f, Y_ENU: %f, Z_ENU: %f\n", state.PositionX, state.PositionY, state.PositionZ)
			}

			if err != nil {
				return nil, fmt.Errorf("ошибка на шаге %d: %v", i, err)
			}

			// Устанавливаем временную метку
			state.Timestamp = data.Timestamp

			// тут добавим преобразования gnss_ENU в геодезические координаты и перевод кватернионов в углы Эйлера
			// все формулы уже имеются - ENUToGeodetic и QuaternionToEuler
			results = append(results, *state)
		}
	}

	return results, nil

}

// initEKF инициализирует Extended Kalman Filter
func (f *Fuzzer) initEKF() error {

	// 1. Создаем модель
	model := models.NewPositionModel(f.cfg)

	// 2. Конфигурация EKF
	ekfConfig := &ekf.EKFConfig{
		TimeStep: f.cfg.EKF.TimeStep,
		InitialState: []float64{
			f.cfg.EKF.InitialState.Position[0],   // X
			f.cfg.EKF.InitialState.Position[1],   // Y
			f.cfg.EKF.InitialState.Position[2],   // Z
			f.cfg.EKF.InitialState.Velocity[0],   // Vx
			f.cfg.EKF.InitialState.Velocity[1],   // Vy
			f.cfg.EKF.InitialState.Velocity[2],   // Vz
			f.cfg.EKF.InitialState.Quaternion[0], // Qw
			f.cfg.EKF.InitialState.Quaternion[1], // Qx
			f.cfg.EKF.InitialState.Quaternion[2], // Qy
			f.cfg.EKF.InitialState.Quaternion[3], // Qz
			f.cfg.EKF.InitialState.Bias_acc[0],   // Bias_ax
			f.cfg.EKF.InitialState.Bias_acc[1],   // Bias_ay
			f.cfg.EKF.InitialState.Bias_acc[2],   // Bias_az
			f.cfg.EKF.InitialState.Bias_gyro[0],  // Bias_wx
			f.cfg.EKF.InitialState.Bias_gyro[1],  // Bias_wy
			f.cfg.EKF.InitialState.Bias_gyro[2],  // Bias_wz
		},
		InitialCov: []float64{
			f.cfg.EKF.InitialCov.Position[0],   // X
			f.cfg.EKF.InitialCov.Position[1],   // Y
			f.cfg.EKF.InitialCov.Position[2],   // Z
			f.cfg.EKF.InitialCov.Velocity[0],   // Vx
			f.cfg.EKF.InitialCov.Velocity[1],   // Vy
			f.cfg.EKF.InitialCov.Velocity[2],   // Vz
			f.cfg.EKF.InitialCov.Quaternion[0], // Qw
			f.cfg.EKF.InitialCov.Quaternion[1], // Qx
			f.cfg.EKF.InitialCov.Quaternion[2], // Qy
			f.cfg.EKF.InitialCov.Quaternion[3], // Qz
			f.cfg.EKF.InitialCov.Bias_acc[0],   // Bias_ax
			f.cfg.EKF.InitialCov.Bias_acc[1],   // Bias_ay
			f.cfg.EKF.InitialCov.Bias_acc[2],   // Bias_az
			f.cfg.EKF.InitialCov.Bias_gyro[0],  // Bias_wx
			f.cfg.EKF.InitialCov.Bias_gyro[1],  // Bias_wy
			f.cfg.EKF.InitialCov.Bias_gyro[2],  // Bias_wz
		},

		ProcessNoise: []float64{
			f.cfg.EKF.ProcessNoise.Position[0],   // X
			f.cfg.EKF.ProcessNoise.Position[1],   // Y
			f.cfg.EKF.ProcessNoise.Position[2],   // Z
			f.cfg.EKF.ProcessNoise.Velocity,      // Vx
			f.cfg.EKF.ProcessNoise.Velocity,      // Vy
			f.cfg.EKF.ProcessNoise.Velocity,      // Vz
			f.cfg.EKF.ProcessNoise.Quaternion[0], // Qw
			f.cfg.EKF.ProcessNoise.Quaternion[1], // Qx
			f.cfg.EKF.ProcessNoise.Quaternion[2], // Qy
			f.cfg.EKF.ProcessNoise.Quaternion[3], // Qz
			f.cfg.EKF.ProcessNoise.Bias_acc,      // Bias_ax
			f.cfg.EKF.ProcessNoise.Bias_acc,      // Bias_ay
			f.cfg.EKF.ProcessNoise.Bias_acc,      // Bias_az
			f.cfg.EKF.ProcessNoise.Bias_gyro,     // Bias_wx
			f.cfg.EKF.ProcessNoise.Bias_gyro,     // Bias_wy
			f.cfg.EKF.ProcessNoise.Bias_gyro,     // Bias_wz
		},

		MeasurementNoise: []float64{
			f.cfg.EKF.MeasurementNoise.Position_GNSS[0], // GNSS_X
			f.cfg.EKF.MeasurementNoise.Position_GNSS[1], // GNSS_Y
			f.cfg.EKF.MeasurementNoise.Position_GNSS[2], // GNSS_Z
			f.cfg.EKF.MeasurementNoise.Speed,            // Speed
		},
	}

	// 3. Создаем EKF
	ekfWrapper, err := ekf.NewEKFWrapper(model, ekfConfig)
	if err != nil {
		return fmt.Errorf("ошибка инициализации EKF: %v", err)
	}

	f.ekf = ekfWrapper

	return nil
}

// initEKF инициализирует Extended Kalman Filter
func (f *Fuzzer) initState(data models.SynchronizedData) error {

	// Инициализировать начальное состояние

	gnssX_ENU, gnssY_ENU, gnssZ_ENU := GeodeticToENU(data.Latitude, data.Longitude, data.Altitude, f.cfg)

	// Оценка перемещения
	f.cfg.EKF.InitialState.Position[0] = gnssX_ENU
	f.cfg.EKF.InitialState.Position[1] = gnssY_ENU
	f.cfg.EKF.InitialState.Position[2] = gnssZ_ENU

	// Оценка скорости (только по координатам GNSS)
	dt := float64(time.Since(f.lastTimeGNSS).Nanoseconds()) / 1e9
	//dt := f.nextTimeGNSS - f.lastTimeGNSS // для отладки

	f.cfg.EKF.InitialState.Velocity[0] = gnssX_ENU / dt
	f.cfg.EKF.InitialState.Velocity[1] = gnssY_ENU / dt
	f.cfg.EKF.InitialState.Velocity[2] = gnssZ_ENU / dt

	// Оценка кватерниона
	// усредненое ускорение или ускорения из последних отсчетов
	Ax0 := f.cfg.EKF.InitialState.Bias_acc[0]
	Ay0 := f.cfg.EKF.InitialState.Bias_acc[1]
	Az0 := f.cfg.EKF.InitialState.Bias_acc[2]
	Ve0 := f.cfg.EKF.InitialState.Velocity[0]
	Vn0 := f.cfg.EKF.InitialState.Velocity[1]

	roll0 := RadiansToDegrees(math.Atan(Ax0 / math.Sqrt(Ay0*Ay0+Az0*Az0)))
	pitch0 := RadiansToDegrees(math.Atan(-Ay0 / math.Sqrt(Ax0*Ax0+Az0*Az0)))
	yaw0 := math.Atan2(Ve0, Vn0)

	q0 := EulerToQuaternion(roll0, pitch0, yaw0)

	f.cfg.EKF.InitialState.Quaternion[0] = q0.W
	f.cfg.EKF.InitialState.Quaternion[1] = q0.X
	f.cfg.EKF.InitialState.Quaternion[2] = q0.Y
	f.cfg.EKF.InitialState.Quaternion[3] = q0.Z

	// Оценка смещений (оцениваются в функции calcBias)
	gRotated := rotateVectorByQuaternion(
		[3]float64{0, 0, -f.gravity},
		q0,
	)

	f.cfg.EKF.InitialState.Bias_acc[0] = Ax0 - gRotated[0]
	f.cfg.EKF.InitialState.Bias_acc[1] = Ay0 - gRotated[1]
	f.cfg.EKF.InitialState.Bias_acc[2] = Az0 - gRotated[2]

	return nil
}

func (f *Fuzzer) calcAverageData(data models.SynchronizedData) {

	// Bias_acc with g // усредненное кажущееся ускорение в g
	f.cfg.EKF.InitialState.Bias_acc[0] += (data.AccelX/float64(f.sizeBufBias) - f.BufBiasAccX[f.sizeBufBias-1]/float64(f.sizeBufBias))
	f.cfg.EKF.InitialState.Bias_acc[1] += (data.AccelY/float64(f.sizeBufBias) - f.BufBiasAccY[f.sizeBufBias-1]/float64(f.sizeBufBias))
	f.cfg.EKF.InitialState.Bias_acc[2] += (data.AccelZ/float64(f.sizeBufBias) - f.BufBiasAccZ[f.sizeBufBias-1]/float64(f.sizeBufBias))

	// Обновление кольцевых буферов Acc
	Rotate(f.BufBiasAccX, 1)
	f.BufBiasAccX[0] = data.AccelX
	Rotate(f.BufBiasAccY, 1)
	f.BufBiasAccY[0] = data.AccelY
	Rotate(f.BufBiasAccZ, 1)
	f.BufBiasAccZ[0] = data.AccelZ

	// Bias_gyro
	f.cfg.EKF.InitialState.Bias_gyro[0] += (data.GyroX/float64(f.sizeBufBias) - f.BufBiasGyroX[f.sizeBufBias-1]/float64(f.sizeBufBias))
	f.cfg.EKF.InitialState.Bias_gyro[1] += (data.GyroY/float64(f.sizeBufBias) - f.BufBiasGyroY[f.sizeBufBias-1]/float64(f.sizeBufBias))
	f.cfg.EKF.InitialState.Bias_gyro[2] += (data.GyroZ/float64(f.sizeBufBias) - f.BufBiasGyroZ[f.sizeBufBias-1]/float64(f.sizeBufBias))

	// Обновление кольцевых буферов Gyro
	Rotate(f.BufBiasGyroX, 1)
	f.BufBiasGyroX[0] = data.GyroX
	Rotate(f.BufBiasGyroY, 1)
	f.BufBiasGyroY[0] = data.GyroY
	Rotate(f.BufBiasGyroZ, 1)
	f.BufBiasGyroZ[0] = data.GyroZ
}

func Rotate(data []float64, pos int) []float64 {
	n := len(data)
	if n == 0 {
		return data
	}

	pos %= n
	if pos < 0 {
		pos += n
	}

	result := make([]float64, n)
	copy(result, data[pos:])
	copy(result[n-pos:], data[:pos])

	return result
}
