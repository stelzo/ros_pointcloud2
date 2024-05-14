extern crate proc_macro;

use std::collections::HashMap;

use proc_macro::TokenStream;
use quote::{quote, ToTokens};
use syn::{parse_macro_input, DeriveInput};

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

// Given a field, get the value of the `rpcl2` renaming attribute like
// #[rpcl2(name = "new_name")]
fn get_ros_fields_attribute(attrs: &[syn::Attribute]) -> Option<syn::Lit> {
    for attr in attrs {
        if attr.path.is_ident("rpcl2") {
            let meta = attr.parse_meta().unwrap();
            if let syn::Meta::List(meta_list) = meta {
                for nested_meta in meta_list.nested {
                    if let syn::NestedMeta::Meta(meta) = nested_meta {
                        if let syn::Meta::NameValue(meta_name_value) = meta {
                            if meta_name_value.path.is_ident("name") {
                                return Some(meta_name_value.lit);
                            }
                        }
                    }
                }
            }
        }
    }
    None
}

fn struct_field_rename_array(input: &DeriveInput) -> Vec<String> {
    let fields = match input.data {
        syn::Data::Struct(ref data) => match data.fields {
            syn::Fields::Named(ref fields) => &fields.named,
            _ => panic!("StructNames can only be derived for structs with named fields"),
        },
        _ => panic!("StructNames can only be derived for structs"),
    };

    fields
        .iter()
        .map(|field| {
            let field_name = field.ident.as_ref().unwrap();
            let ros_fields_attr = get_ros_fields_attribute(&field.attrs);
            match ros_fields_attr {
                Some(ros_fields) => match ros_fields {
                    syn::Lit::Str(lit_str) => {
                        let val = lit_str.value();
                        if val.is_empty() {
                            panic!("Empty string literals are not allowed for the rpcl2 attribute");
                        }
                        val
                    }
                    _ => {
                        panic!("Only string literals are allowed for the rpcl2 attribute")
                    }
                },
                None => String::from(field_name.to_token_stream().to_string()),
            }
        })
        .collect()
}

/// This macro will implement the `Fields` trait for your struct so you can use your point for the PointCloud2 conversion.
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

/// This macro will fully implement the `PointConvertible` trait for your struct so you can use your point for the PointCloud2 conversion.
///
/// Note that the repr(C) attribute is required for the struct to work efficiently with C++ PCL.
/// With Rust layout optimizations, the struct might not work with the PCL library but the message still conforms to the description of PointCloud2.
/// Furthermore, Rust layout can lead to smaller messages to be send over the network.
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
