package models

import (
	"time" // для реального случая

	"gonum.org/v1/gonum/mat"
	"main.go/config"
)

// PositionModel реализует модель для EKF
type PositionModel struct {
	stateDim  int // Размерность состояния
	inputDim  int // Размерность входа
	outputDim int // Размерность выхода

	config *config.Config

	// Вспомогательные переменные
	gravity float64

	LastTimeIMU time.Time // последнее обновление IMU // для реального случая
	dT          float64
}

// NewPositionModel создает новую модель позиционирования
func NewPositionModel(cfg *config.Config) *PositionModel {
	return &PositionModel{
		stateDim:  cfg.EKF.StateSize,       // [x, y, z, vx, vy, vz, qw, qx, qy, qz, bias_ax, bias_ay, bias_az, bias_wx, bias_wy, bias_wz] 		// n: размер состояния
		inputDim:  6,                       // [ax_measured, ay_measured, az_measured, wx_measured, wy_measured, wz_measured] 					// p: размер управления (ax, ay, az, wx, wy, wz, dt)
		outputDim: cfg.EKF.MeasurementSize, // [x_measured, y_measured, z_measured, v_measured] // m: размер измерений
		gravity:   9.81,

		LastTimeIMU: time.Now(), // последнее обновление IMU // для реального случая
		dT:          0,          // период обновления IMU // Для отладки
	}
}

// updateData актуализирует время обновления данных
func (m *PositionModel) UpdateData() {
	//m.dT = 1 / 10.0 // для отладки !!!!! ввести свою частоту IMU
	m.dT = float64(time.Since(m.LastTimeIMU).Nanoseconds()) / 1e9 // для реального случая
	m.LastTimeIMU = time.Now()                                    // для реального случая
}

// SystemDims возвращает размерности системы
func (m *PositionModel) SystemDims() (int, int, int, int) {
	return m.stateDim, m.inputDim, m.outputDim, 0
}

// Propagate предсказывает следующее состояние
func (m *PositionModel) Propagate(x, u, w mat.Vector) (mat.Vector, error) {
	// x: [x, y, z, vx, vy, vz, roll, pitch, yaw, ax_bias, ay_bias, az_bias, wx_bias, wy_bias, wz_bias]
	// u: [ax, ay, az, wx, wy, wz]

	dt := m.dT

	// 1. Вычисляем следующее состояние
	n := x.Len()
	xNext := mat.NewVecDense(n, nil)

	// 2. Извлечение управления с компенсацией смещений
	ax := u.AtVec(0) - x.AtVec(10) // ускорение X (с компенсацией смещения)
	ay := u.AtVec(1) - x.AtVec(11) // ускорение Y (с компенсацией смещения)
	az := u.AtVec(2) - x.AtVec(12) // ускорение Z (с компенсацией смещения)
	wx := u.AtVec(3) - x.AtVec(13) // угловая скорость X (с компенсацией смещения)
	wy := u.AtVec(4) - x.AtVec(14) // угловая скорость Y (с компенсацией смещения)
	wz := u.AtVec(5) - x.AtVec(15) // угловая скорость Z (с компенсацией смещения)

	// 3. Преобразование ускорений из локальной системы датчика в глобальную систему координат (ENU) с помощью кватерниона
	// Извлечение кватерниона ориентации из текущего состояния
	q := Quaternion{
		W: x.AtVec(6),
		X: x.AtVec(7),
		Y: x.AtVec(8),
		Z: x.AtVec(9),
	}

	accRotated := rotateVectorByQuaternion(
		[3]float64{ax, ay, az},
		q,
	)

	// 4. Интегрирование ускорений для получения скорости
	newVx := x.AtVec(3) + accRotated[0]*dt
	newVy := x.AtVec(4) + accRotated[1]*dt
	newVz := x.AtVec(5) + (accRotated[2]-m.gravity)*dt

	// 5. Интегрирование скорости для получения позиции
	newX := x.AtVec(0) + x.AtVec(3)*dt + 0.5*accRotated[0]*dt*dt
	newY := x.AtVec(1) + x.AtVec(4)*dt + 0.5*accRotated[1]*dt*dt
	newZ := x.AtVec(2) + x.AtVec(5)*dt + 0.5*(accRotated[2]-m.gravity)*dt*dt

	// 6. Обновление ориентации с помощью кватернионов
	// Вычисляем дельта-кватернион из угловой скорости
	deltaQ := calculateDeltaQuaternion(wx, wy, wz, dt)

	// Обновляем кватернион: q_new = q ⊗ delta_q
	qNew := quaternionMultiply(q, deltaQ)

	// Нормализация кватерниона
	qNew = normalizeQuaternion(qNew)

	// 7. Обновление состояния
	xNext.SetVec(0, newX)
	xNext.SetVec(1, newY)
	xNext.SetVec(2, newZ)
	xNext.SetVec(3, newVx)
	xNext.SetVec(4, newVy)
	xNext.SetVec(5, newVz)
	xNext.SetVec(6, qNew.W)
	xNext.SetVec(7, qNew.X)
	xNext.SetVec(8, qNew.Y)
	xNext.SetVec(9, qNew.Z)
	xNext.SetVec(10, x.AtVec(10)) // смещения считаем постоянными
	xNext.SetVec(11, x.AtVec(11))
	xNext.SetVec(12, x.AtVec(12))
	xNext.SetVec(13, x.AtVec(13)) // смещения считаем постоянными
	xNext.SetVec(14, x.AtVec(14))
	xNext.SetVec(14, x.AtVec(15))

	// 8. Добавляем шум процесса
	if w != nil {
		xNext.AddVec(xNext, w)
	}

	return xNext, nil
}

// Observe возвращает наблюдаемые величины
func (m *PositionModel) Observe(x, u, v mat.Vector) (mat.Vector, error) {

	// 1. Вычисляем следующее измерение
	y := mat.NewVecDense(4, nil)

	// 2. Извлечение скоростей из текущего состояния
	Vx := x.AtVec(3)
	Vy := x.AtVec(4)
	Vz := x.AtVec(5)

	// 3. Извлечение кватерниона ориентации из текущего состояния и преобразуем его в сопряженный для перевода скорости из системы ENU в систему объекта
	// q_ : ENU -> car
	q_ := Quaternion{
		W: x.AtVec(6),
		X: -x.AtVec(7),
		Y: -x.AtVec(8),
		Z: -x.AtVec(9),
	}

	// 4. GNSS уже преобразован в метры в системе ENU
	y.SetVec(0, x.AtVec(0))
	y.SetVec(1, x.AtVec(1))
	y.SetVec(2, x.AtVec(2))

	// 5. Вычисление скорости спидометра
	velRotated := rotateVectorByQuaternion(
		[3]float64{Vx, Vy, Vz},
		q_,
	)

	y.SetVec(3, velRotated[1])

	// 6. Добавляем шум измерений
	if v != nil {
		y.AddVec(y, v)
	}

	return y, nil
}
