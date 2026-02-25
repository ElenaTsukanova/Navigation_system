package models

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

// calculateDeltaQuaternion вычисляет дельта-кватернион из угловой скорости
func calculateDeltaQuaternion(
	wx, wy, wz, dt float64,
) Quaternion {
	// Норма угловой скорости
	omegaNorm := math.Sqrt(wx*wx + wy*wy + wz*wz)

	if omegaNorm < 1e-12 {
		// Нет вращения - единичный кватернион
		return Quaternion{W: 1, X: 0, Y: 0, Z: 0}
	}

	// Угол поворота
	theta := omegaNorm * dt

	// Нормализованная ось вращения
	axisX := wx / omegaNorm
	axisY := wy / omegaNorm
	axisZ := wz / omegaNorm

	// Дельта-кватернион
	sinHalfTheta := math.Sin(theta / 2)
	cosHalfTheta := math.Cos(theta / 2)

	return Quaternion{
		W: cosHalfTheta,
		X: axisX * sinHalfTheta,
		Y: axisY * sinHalfTheta,
		Z: axisZ * sinHalfTheta,
	}
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

// normalizeQuaternion нормализует кватернион
func normalizeQuaternion(q Quaternion) Quaternion {
	norm := math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)

	if norm < 1e-12 {
		return Quaternion{W: 1, X: 0, Y: 0, Z: 0}
	}

	return Quaternion{
		W: q.W / norm,
		X: q.X / norm,
		Y: q.Y / norm,
		Z: q.Z / norm,
	}
}
