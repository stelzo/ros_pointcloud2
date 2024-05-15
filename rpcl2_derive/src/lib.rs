extern crate proc_macro;

use std::collections::HashMap;

use proc_macro::TokenStream;
use quote::{quote, ToTokens};
use syn::{parenthesized, parse_macro_input, DeriveInput, LitStr};

fn get_allowed_types() -> HashMap<&'static str, usize> {
    let mut allowed_datatypes = HashMap::<&'static str, usize>::new();
    allowed_datatypes.insert("f32", std::mem::size_of::<f32>());
    allowed_datatypes.insert("f64", std::mem::size_of::<f64>());
    allowed_datatypes.insert("i32", std::mem::size_of::<i32>());
    allowed_datatypes.insert("u8", std::mem::size_of::<u8>());
    allowed_datatypes.insert("u16", std::mem::size_of::<u16>());
    allowed_datatypes.insert("u32", std::mem::size_of::<u32>());
    allowed_datatypes.insert("i8", std::mem::size_of::<i8>());
    allowed_datatypes.insert("i16", std::mem::size_of::<i16>());
    allowed_datatypes
}

fn struct_field_rename_array(input: &DeriveInput) -> Vec<String> {
    let fields = match input.data {
        syn::Data::Struct(ref data) => match data.fields {
            syn::Fields::Named(ref fields) => &fields.named,
            _ => panic!("StructNames can only be derived for structs with named fields"),
        },
        _ => panic!("StructNames can only be derived for structs"),
    };

    let mut field_names = Vec::with_capacity(fields.len());
    for f in fields.iter() {
        if f.attrs.len() == 0 {
            field_names.push(f.ident.as_ref().unwrap().to_token_stream().to_string());
        } else {
            f.attrs.iter().for_each(|attr| {
                if attr.path().is_ident("rpcl2") {
                    let res = attr.parse_nested_meta(|meta| {
                        if meta.path.is_ident("rename") {
                            let new_name;
                            parenthesized!(new_name in meta.input);
                            let lit: LitStr = new_name.parse()?;
                            field_names.push(lit.value());
                            Ok(())
                        } else {
                            panic!("expected `name` attribute");
                        }
                    });
                    if let Err(err) = res {
                        panic!("Error parsing attribute: {}", err);
                    }
                }
            });
        }
    }

    field_names
}

/// This macro implements the `Fields` trait which is a subset of the `PointConvertible` trait.
/// It is useful for points that convert the `From` trait themselves but want to use this macro for not repeating the field names.
///
/// You can rename the fields with the `rename` attribute.
///
/// Use the rename attribute if your struct field name should be different to the ROS field name.
#[proc_macro_derive(Fields, attributes(rpcl2))]
pub fn ros_point_fields_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let struct_name = &input.ident;

    let field_names = struct_field_rename_array(&input)
        .into_iter()
        .map(|field_name| {
            quote! { #field_name }
        });

    let field_names_len = field_names.len();

    let expanded = quote! {
        impl Fields<#field_names_len> for #struct_name {
            fn field_names_ordered() -> [&'static str; #field_names_len] {
                [
                    #(#field_names,)*
                ]
            }
        }
    };

    // Return the generated implementation
    expanded.into()
}

/// This macro implements the `PointConvertible` trait for your struct so you can use your point for the PointCloud2 conversion.
///
/// The struct field names are used in the message if you do not use the `rename` attribute for a custom name.
///
/// Note that the repr(C) attribute is required for the struct to work efficiently with C++ PCL.
/// With Rust layout optimizations, the struct might not work with the PCL library but the message still conforms to the description of PointCloud2.
/// Furthermore, Rust layout can lead to smaller messages to be send over the network.
///
#[proc_macro_derive(PointConvertible, attributes(rpcl2))]
pub fn ros_point_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = input.clone().ident;

    let fields = match input.data {
        syn::Data::Struct(ref data) => data.fields.clone(),
        _ => {
            return syn::Error::new_spanned(input, "Only structs are supported")
                .to_compile_error()
                .into()
        }
    };

    let allowed_datatypes = get_allowed_types();

    if fields.is_empty() {
        return syn::Error::new_spanned(input, "No fields found")
            .to_compile_error()
            .into();
    }

    for field in fields.iter() {
        let ty = field.ty.to_token_stream().to_string();
        if !allowed_datatypes.contains_key(&ty.as_str()) {
            return syn::Error::new_spanned(field, "Field type not allowed")
                .to_compile_error()
                .into();
        }
    }

    let field_len_token: usize = fields.len();

    let field_names = struct_field_rename_array(&input)
        .into_iter()
        .map(|field_name| {
            quote! { #field_name }
        });

    let field_impl = quote! {
        impl ros_pointcloud2::Fields<#field_len_token> for #name {
            fn field_names_ordered() -> [&'static str; #field_len_token] {
                [
                    #(#field_names,)*
                ]
            }
        }
    };

    let field_names_get = fields
        .iter()
        .enumerate()
        .map(|(idx, f)| {
            let field_name = f.ident.as_ref().unwrap();
            quote! { #field_name: point[#idx].get() }
        })
        .collect::<Vec<_>>();

    let from_my_point = quote! {
        impl From<ros_pointcloud2::RPCL2Point<#field_len_token>> for #name {
            fn from(point: ros_pointcloud2::RPCL2Point<#field_len_token>) -> Self {
                Self {
                    #(#field_names_get,)*
                }
            }
        }
    };

    let field_names_into = fields
        .iter()
        .map(|f| {
            let field_name = f.ident.as_ref().unwrap();
            quote! { point.#field_name.into() }
        })
        .collect::<Vec<_>>();

    let from_custom_point = quote! {
        impl From<#name> for ros_pointcloud2::RPCL2Point<#field_len_token> {
            fn from(point: #name) -> Self {
                [ #(#field_names_into,)* ].into()
            }
        }
    };

    let convertible = quote! {
        impl ros_pointcloud2::PointConvertible<#field_len_token> for #name {}
    };

    let out = TokenStream::from(quote! {
        #field_impl
        #from_my_point
        #from_custom_point
        #convertible
    });

    TokenStream::from(out)
}
