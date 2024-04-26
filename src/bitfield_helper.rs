#[macro_export]
macro_rules! impl_enum_bitrange {
    ($enum:ident {
        $(
            $variant:ident = $value:expr,
        )*
    }) => {
        impl BitRange<$enum> for u8 {
            fn bit_range(&self, msb: usize, lsb: usize) -> $enum {
                let mask = ((1 << (msb - lsb + 1)) - 1) << lsb;
                match (*self & mask) >> lsb {
                    $(
                        $value => $enum::$variant,
                    )*
                    _ => panic!("Invalid bit range for enum"),
                }
            }
        }

        impl BitRangeMut<$enum> for u8 {
            fn set_bit_range(&mut self, msb: usize, lsb: usize, value: $enum) {
                let mask = !(((1 << (msb - lsb + 1)) - 1) << lsb);
                *self &= mask;
                *self |= (value as u8) << lsb;
            }
        }
    };
}

#[macro_export]
macro_rules! define_enum_with_bitrange {
    ($name:ident {
        $(
            $(#[$meta:meta])*
            $variant:ident = $value:expr,
        )*
    }) => {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        #[repr(u8)]
        pub enum $name {
            $(
                $(#[$meta])*
                $variant = $value,
            )*
        }

        $crate::impl_enum_bitrange!($name {
            $($variant = $value,)*
        });
    };
}
