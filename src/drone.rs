// use alloc::rc::Rc;
// use core::borrow::BorrowMut;
//
// use elinalgebra::Vector2;
//
// pub trait Observer<E> {
//     fn notify(&mut self, event: E);
// }
//
// impl<E> Subject<E> {
//     pub fn notify(&mut self, event: E) {
//         self.observers.iter_mut().for_each(|o|
//             o.borrow_mut().notify(event))
//     }
// }
//
// pub struct Subject<'a, E> {
//     observers: [Rc<&'a mut dyn Observer<E>>],
// }
//
//
//
// pub trait Controls {
//     fn set_direction(&mut self, dir: &Vector2<i32>);
//     fn get_direction_to(&self, dst: &mut Vector2<i32>);
//
//     fn get_direction(&self) -> Vector2<i32> {
//         let mut dst = Vector2 { x: 0, y: 0 };
//         self.get_direction_to(&mut dst);
//         dst
//     }
// }
//
// pub trait Gyroscope<E> {
//     fn gyro_acc_to(&mut self, dst: &mut Vector2<f32>) -> Result<(), E>;
//
//     fn gyro_acc(&mut self) -> Result<Vector2<f32>, E> {
//         let mut dst = Vector2 { x: 0f32, y: 0f32 };
//         self.gyro_acc_to(&mut dst)?;
//         Ok(dst)
//     }
//
//     fn planar_acc_to(&mut self, dst: &mut Vector2<f32>) -> Result<(), E>;
//
//     fn planar_acc(&mut self) -> Result<Vector2<f32>, E> {
//         let mut dst = Vector2 { x: 0f32, y: 0f32 };
//         self.planar_acc_to(&mut dst)?;
//         Ok(dst)
//     }
// }