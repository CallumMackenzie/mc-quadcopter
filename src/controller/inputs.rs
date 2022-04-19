// enum InputError<T> {}

// enum DroneMode {
//     StablePosition,
//     StableRotation,
//     GeneralStable,
// }

// struct RemoteInputs<'a> {
//     pub mode: Rc<DroneMode>,
// }

// impl<'a, E> RemoteInputs<'a> {
//     fn init() -> Result<(), InputError<E>> {
//         Ok(())
//     }
// }

// struct LocalInputs<'a> {
//     pub mode: Rc<DroneMode>,
// }

// impl<'a, E> LocalInputs<'a> {
//     fn init() -> Result<(), InputError<E>> {
//         Ok(())
//     }
// }

// pub struct Inputs<'a> {
//     local: LocalInputs<'a>,
//     remote: RemoteInputs<'a>,
//     pub mode: Rc<DroneMode>,
// }

// impl<'a, E> Inputs<'a>
// where
//     E: Sized,
// {
//     fn new(mode: &Rc<DroneMode>) {
//         Inputs {
//             mode: Rc::clone(mode),
//             local: LocalInputs {
//                 mode: Rc::clone(mode),
//             },
//             remote: RemoteInputs {
//                 mode: Rc::clone(mode),
//             },
//         }
//     }

//     fn init(&mut self) -> Result<(), InputError<E>> {
//         self.local.init()?;
//         self.remote.init()?;
//         Ok(())
//     }
// }
