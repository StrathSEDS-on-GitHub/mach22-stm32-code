use core::{future::Future, pin};



pub struct NbFuture<T, E, F>
where
    F: FnMut() -> nb::Result<T, E> + Unpin,
{
    function: F,
}

impl<T, E, F> Future for NbFuture<T, E, F>
where
    F: FnMut() -> nb::Result<T, E> + Unpin,
{
    type Output = Result<T, E>;

    fn poll(
        mut self: pin::Pin<&mut Self>,
        _cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        match (self.function)() {
            Ok(value) => core::task::Poll::Ready(Ok(value)),
            Err(nb::Error::WouldBlock) => core::task::Poll::Pending,
            Err(nb::Error::Other(e)) => core::task::Poll::Ready(Err(e)),
        }
    }
}

impl<T, E, F> NbFuture<T, E, F>
where
    F: FnMut() -> nb::Result<T, E> + Unpin,
{
    pub fn new(function: F) -> Self {
        NbFuture { function }
    }
}
