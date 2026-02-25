package fuzzer

import (
	"math"
)

// Quaternion структура для представления кватерниона
type Quaternion struct {
	W, X, Y, Z float64
}

// rotateVectorByQuaternion вращает вектор с помощью кватерниона
func rotateVectorByQuaternion(v [3]float64, q Quaternion) [3]float64 {
	// Кватернионное умножение: v' = q ⊗ v ⊗ q*
	// где v - "чистый" кватернион (0, vx, vy, vz), q* - сопряженный кватернион

	// Представляем вектор как чистый кватернион
	vQuat := Quaternion{W: 0, X: v[0], Y: v[1], Z: v[2]}

	// Вычисляем q* (сопряженный)
	qConj := Quaternion{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}

	// v' = q ⊗ v ⊗ q*
	temp := quaternionMultiply(q, vQuat)
	result := quaternionMultiply(temp, qConj)

	return [3]float64{result.X, result.Y, result.Z}
}

// quaternionMultiply умножает два кватерниона
func quaternionMultiply(q1, q2 Quaternion) Quaternion {
	return Quaternion{
		W: q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z,
		X: q1.W*q2.X + q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y,
		Y: q1.W*q2.Y - q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X,
		Z: q1.W*q2.Z + q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W,
	}
}

// EulerToQuaternionCar - конвертация для автомобильной системы координат
// Порядок вращений: Z (Yaw) -> X (Pitch) -> Y (Roll)
func EulerToQuaternion(yaw, pitch, roll float64) Quaternion {
	// Половинные углы
	hy := yaw * 0.5
	hp := pitch * 0.5
	hr := roll * 0.5

	// Синусы и косинусы
	cy := math.Cos(hy)
	sy := math.Sin(hy)
	cp := math.Cos(hp)
	sp := math.Sin(hp)
	cr := math.Cos(hr)
	sr := math.Sin(hr)

	// Композиция: сначала Yaw вокруг Z, потом Pitch вокруг X, потом Roll вокруг Y
	// Это дает кватернион: q = qRollY * qPitchX * qYawZ

	return Quaternion{
		W: cy*cp*cr + sy*sp*sr,
		X: cy*sp*cr + sy*cp*sr,
		Y: sy*cp*cr - cy*sp*sr,
		Z: cy*cp*sr - sy*sp*cr,
	}
}

// QuaternionToEulerCar - обратное преобразование
func QuaternionToEuler(qw, qx, qy, qz float64) (yaw, pitch, roll float64) {
	// Yaw (вращение вокруг Z)
	siny_cosp := 2 * (qw*qz + qx*qy)
	cosy_cosp := 1 - 2*(qy*qy+qz*qz)
	yaw = math.Atan2(siny_cosp, cosy_cosp)

	// Pitch (вращение вокруг X)
	sinp := 2 * (qw*qx - qy*qz)
	if math.Abs(sinp) >= 1 {
		pitch = math.Copysign(math.Pi/2, sinp)
	} else {
		pitch = math.Asin(sinp)
	}

	// Roll (вращение вокруг Y)
	sinr_cosp := 2 * (qw*qy - qx*qz)
	cosr_cosp := 1 - 2*(qx*qx+qy*qy)
	roll = math.Atan2(sinr_cosp, cosr_cosp)

	return
}
