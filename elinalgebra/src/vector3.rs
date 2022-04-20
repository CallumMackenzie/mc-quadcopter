use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};
use ufmt::derive::uDebug;

#[derive(Copy, Clone, Debug, uDebug)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

impl<T: Copy> Vector3<T> {
    pub fn filled(v: T) -> Self {
        Self::new(v, v, v)
    }
}

impl<T> Mul<f32> for Vector3<T>
where T: Mul<f32, Output = T>
{
	type Output = Self;
	fn mul(self, o: f32) -> Self::Output {
		Self {
			x: self.x * o,
			y: self.y * o,
			z: self.z * o,
		}
	}
}

impl<T> DivAssign<f32> for Vector3<T>
where
    T: DivAssign<f32>,
{
    fn div_assign(&mut self, o: f32) {
        self.x /= o;
        self.y /= o;
        self.z /= o;
    }
}

impl<T> MulAssign<f32> for Vector3<T>
where
    T: MulAssign<f32>,
{
    fn mul_assign(&mut self, o: f32) {
        self.x *= o;
        self.y *= o;
        self.z *= o;
    }
}

impl<T> AddAssign<f32> for Vector3<T>
where
    T: AddAssign<f32>,
{
    fn add_assign(&mut self, o: f32) {
        self.x += o;
        self.y += o;
        self.z += o;
    }
}

impl<T> SubAssign<f32> for Vector3<T>
where
    T: SubAssign<f32>,
{
    fn sub_assign(&mut self, o: f32) {
        self.x -= o;
        self.y -= o;
        self.z -= o;
    }
}

impl<T> Add for Vector3<T>
where
    T: Add<T, Output = T>,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl<T> Sub for Vector3<T>
where
    T: Sub<T, Output = T>,
{
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl<T> Mul for Vector3<T>
where
    T: Mul<T, Output = T>,
{
    type Output = Self;
    fn mul(self, other: Self) -> Self::Output {
        Self {
            x: self.x * other.x,
            y: self.y * other.y,
            z: self.z * other.z,
        }
    }
}

impl<T> Div for Vector3<T>
where
    T: Div<T, Output = T>,
{
    type Output = Self;
    fn div(self, other: Self) -> Self::Output {
        Self {
            x: self.x / other.x,
            y: self.y / other.y,
            z: self.z / other.z,
        }
    }
}

impl<T> AddAssign for Vector3<T>
where
    T: AddAssign<T>,
{
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl<T> SubAssign for Vector3<T>
where
    T: SubAssign<T>,
{
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}

impl<T> MulAssign for Vector3<T>
where
    T: MulAssign<T>,
{
    fn mul_assign(&mut self, other: Self) {
        self.x *= other.x;
        self.y *= other.y;
        self.z *= other.z;
    }
}

impl<T> DivAssign for Vector3<T>
where
    T: DivAssign<T>,
{
    fn div_assign(&mut self, other: Self) {
        self.x /= other.x;
        self.y /= other.y;
        self.z /= other.z;
    }
}
