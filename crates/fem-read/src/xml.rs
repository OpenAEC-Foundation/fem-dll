//! The few XML elements the VTK collection files name their files in.

use std::collections::HashMap;

/// The attributes of every element named `tag`, in the order of the file.
pub fn elements(data: &[u8], tag: &[u8]) -> Result<Vec<HashMap<String, String>>, String> {
    use quick_xml::events::Event;
    let mut reader = quick_xml::Reader::from_reader(data);
    let mut buf = Vec::new();
    let mut out = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) if e.local_name().as_ref() == tag => {
                let mut attributes = HashMap::new();
                for a in e.attributes().flatten() {
                    let key = String::from_utf8_lossy(a.key.local_name().as_ref()).into_owned();
                    let value = a.normalized_value(quick_xml::XmlVersion::Implicit1_0).map(|v| v.into_owned()).unwrap_or_else(|_| String::from_utf8_lossy(&a.value).into_owned());
                    attributes.insert(key, value);
                }
                out.push(attributes);
            }
            Ok(Event::Eof) => break,
            Ok(_) => {}
            Err(e) => return Err(format!("not XML it can read: {e}")),
        }
        buf.clear();
    }
    Ok(out)
}
