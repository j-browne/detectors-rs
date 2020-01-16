use raumlehre::config::Config;
use std::io::Cursor;

#[test]
fn empty_object() {
    let mut config = Config::new();
    let cursor = Cursor::new(include_str!("configurations/empty_object.json"));
    assert!(config.add_from_reader(cursor).is_ok());
}

#[test]
fn empty_fields() {
    let mut config = Config::new();
    let cursor = Cursor::new(include_str!("configurations/empty_fields.json"));
    assert!(config.add_from_reader(cursor).is_ok());
}

#[test]
fn end_cap() {
    let mut config = Config::new();
    let cursor = Cursor::new(include_str!("configurations/end_cap.json"));
    assert!(config.add_from_reader(cursor).is_ok());
}

#[test]
fn empty() {
    let mut config = Config::new();
    let cursor = Cursor::new(include_str!("configurations/empty.json"));
    assert!(config.add_from_reader(cursor).is_err());
}

#[test]
fn invalid_field() {
    let mut config = Config::new();
    let cursor = Cursor::new(include_str!("configurations/invalid_field.json"));
    assert!(config.add_from_reader(cursor).is_err());
}
