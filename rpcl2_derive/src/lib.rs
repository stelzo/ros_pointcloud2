extern crate proc_macro;

use std::collections::HashMap;

use proc_macro::TokenStream;
use quote::{quote, ToTokens};
use syn::{parse_macro_input, DeriveInput};

fn get_allowed_types() -> HashMap<&'static str, usize> {
    let mut allowed_datatypes = HashMap::<&'static str, usize>::new();
    allowed_datatypes.insert("f32", 4);
    allowed_datatypes.insert("f64", 8);
    allowed_datatypes.insert("i32", 4);
    allowed_datatypes.insert("u8", 1);
    allowed_datatypes.insert("u16", 2);
    allowed_datatypes.insert("u32", 4);
    allowed_datatypes.insert("i8", 1);
    allowed_datatypes.insert("i16", 2);
    allowed_datatypes
}

/// Derive macro for the `Fields` trait.
/// 
/// Given the ordering from the source code of your struct, this macro will generate an array of field names.
#[proc_macro_derive(RosFields)]
pub fn ros_point_fields_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = input.clone().ident;

    let fields = match input.data {
        syn::Data::Struct(ref data) => data.fields.clone(),
        _ => return syn::Error::new_spanned(input, "Only structs are supported").to_compile_error().into(),
    };

    let allowed_datatypes = get_allowed_types();

    if fields.is_empty() {
        return syn::Error::new_spanned(input, "No fields found").to_compile_error().into();
    }

    for field in fields.iter() {
        let ty = field.ty.to_token_stream().to_string();
        if !allowed_datatypes.contains_key(&ty.as_str()) {
            return syn::Error::new_spanned(field, "Field type not allowed").to_compile_error().into();
        }
    }

    let field_len_token: usize = fields.len();
    
    let field_names = fields.iter().map(|field| {
        let field_name = field.ident.as_ref().unwrap();
        quote! { stringify!(#field_name) }
    }).collect::<Vec<_>>();

    let field_impl = quote! {
        impl Fields<#field_len_token> for #name {
            fn field_names_ordered() -> [&'static str; #field_len_token] {
                [
                    #(#field_names,)*
                ]
            }
        }
    };

    TokenStream::from(field_impl)
}

/// This macro will fully implement the `PointConvertible` trait for your struct so you can use your point for the PointCloud2 conversion.
///
/// Note that the repr(C) attribute is required for the struct to work efficiently with C++ PCL.
/// With Rust layout optimizations, the struct might not work with the PCL library but the message still conforms to the specification of PointCloud2.
/// Furthermore, Rust layout can lead to smaller messages to be send over the network.
#[proc_macro_derive(RosFull)]
pub fn ros_point_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = input.clone().ident;

    let fields = match input.data {
        syn::Data::Struct(ref data) => data.fields.clone(),
        _ => return syn::Error::new_spanned(input, "Only structs are supported").to_compile_error().into(),
    };

    let allowed_datatypes = get_allowed_types();

    if fields.is_empty() {
        return syn::Error::new_spanned(input, "No fields found").to_compile_error().into();
    }

    for field in fields.iter() {
        let ty = field.ty.to_token_stream().to_string();
        if !allowed_datatypes.contains_key(&ty.as_str()) {
            return syn::Error::new_spanned(field, "Field type not allowed").to_compile_error().into();
        }
    }

    let field_len_token: usize = fields.len();
    
    let field_names = fields.iter().map(|field| {
        let field_name = field.ident.as_ref().unwrap();
        quote! { stringify!(#field_name) }
    }).collect::<Vec<_>>();

    let field_impl = quote! {
        impl ros_pointcloud2::Fields<#field_len_token> for #name {
            fn field_names_ordered() -> [&'static str; #field_len_token] {
                [
                    #(#field_names,)*
                ]
            }
        }
    };

    let field_names_get = fields.iter().enumerate().map(|(idx, f)| {
        let field_name = f.ident.as_ref().unwrap();
        quote! { #field_name: point.fields[#idx].get() }
    }).collect::<Vec<_>>();

    let from_my_point =  quote! {
        impl From<ros_pointcloud2::RPCL2Point<#field_len_token>> for #name {
            fn from(point: ros_pointcloud2::RPCL2Point<#field_len_token>) -> Self {
                Self {
                    #(#field_names_get,)*
                }
            }
        }
    };

    let field_names_into = fields.iter().map(|f| {
        let field_name = f.ident.as_ref().unwrap();
        quote! { point.#field_name.into() }
    }).collect::<Vec<_>>();

    let from_custom_point = quote! {
        impl From<#name> for ros_pointcloud2::RPCL2Point<#field_len_token> {
            fn from(point: #name) -> Self {
                ros_pointcloud2::RPCL2Point {
                    fields: [ #(#field_names_into,)* ]
                }
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