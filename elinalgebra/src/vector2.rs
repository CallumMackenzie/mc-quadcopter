use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

#[derive(Copy, Clone, Debug)]
pub struct Vector2<T> {
    pub x: T,
    pub y: T,
}

impl<T> Vector2<T> {
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl<T: Copy> Vector2<T> {
    pub fn filled(v: T) -> Self {
        Self::new(v, v)
    }
}

impl<T> DivAssign<f32> for Vector2<T>
where T: DivAssign<f32>
{
	fn div_assign(&mut self, o: f32) {
		self.x /= o;
		self.y /= o;
	}
}

impl<T> MulAssign<f32> for Vector2<T>
where T: MulAssign<f32>
{
	fn mul_assign(&mut self, o: f32) {
		self.x *= o;
		self.y *= o;
	}
}

impl<T> AddAssign<f32> for Vector2<T>
where T: AddAssign<f32>
{
	fn add_assign(&mut self, o: f32) {
		self.x += o;
		self.y += o;
	}
}

impl<T> SubAssign<f32> for Vector2<T>
where T: SubAssign<f32>
{
	fn sub_assign(&mut self, o: f32) {
		self.x -= o;
		self.y -= o;
	}
}

impl<T> Add for Vector2<T>
where
    T: Add<T, Output = T>,
{
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl<T> Sub for Vector2<T>
where
    T: Sub<T, Output = T>,
{
    type Output = Self;
    fn sub(self, other: Self) -> Self::Output {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl<T> Mul for Vector2<T>
where
    T: Mul<T, Output = T>,
{
    type Output = Self;
    fn mul(self, other: Self) -> Self::Output {
        Self {
            x: self.x * other.x,
            y: self.y * other.y,
        }
    }
}

impl<T> Div for Vector2<T>
where
    T: Div<T, Output = T>,
{
    type Output = Self;
    fn div(self, other: Self) -> Self::Output {
        Self {
            x: self.x / other.x,
            y: self.y / other.y,
        }
    }
}

impl<T> AddAssign for Vector2<T>
where
    T: AddAssign<T>,
{
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl<T> SubAssign for Vector2<T>
where
    T: SubAssign<T>,
{
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl<T> MulAssign for Vector2<T>
where
    T: MulAssign<T>,
{
    fn mul_assign(&mut self, other: Self) {
        self.x *= other.x;
        self.y *= other.y;
    }
}

impl<T> DivAssign for Vector2<T>
where
    T: DivAssign<T>,
{
    fn div_assign(&mut self, other: Self) {
        self.x /= other.x;
        self.y /= other.y;
    }
}
