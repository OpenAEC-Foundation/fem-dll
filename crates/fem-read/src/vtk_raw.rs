//! XML VTK files whose appended data is raw bytes (as most programs write them), turned into the
//! same file with that data in base64, which `vtkio` reads.
//!
//! Each data array of appended data starts at its `offset` with a header of numbers of
//! `header_type` (UInt32 unless the file says UInt64): uncompressed, the number of bytes that
//! follow; compressed, the number of blocks, the block size, the size of the last block and the
//! compressed size of each block, then the blocks. In base64, the header and the data of an
//! uncompressed array are one run of base64, and of a compressed array the header and the
//! blocks are each a run; the offsets count the base64 characters.

use base64::Engine;

fn find(data: &[u8], what: &[u8], from: usize) -> Option<usize> {
    data.get(from..)?.windows(what.len()).position(|w| w == what).map(|k| k + from)
}

/// The value of attribute `name` in the first element named `tag` of `head`.
fn attribute<'a>(head: &'a [u8], tag: &[u8], name: &[u8]) -> Option<&'a [u8]> {
    let start = find(head, tag, 0)?;
    let end = find(head, b">", start)?;
    let element = &head[start..end];
    let mut key = name.to_vec();
    key.extend_from_slice(b"=\"");
    let at = find(element, &key, 0)? + key.len();
    let close = find(element, b"\"", at)?;
    Some(&element[at..close])
}

/// The file with its raw appended data in base64, or `None` when it has none (or it is not
/// laid out as described, in which case the file is read as it is).
pub fn to_base64(data: &[u8]) -> Option<Vec<u8>> {
    let tag = find(data, b"<AppendedData", 0)?;
    let tag_end = find(data, b">", tag)?;
    if find(&data[tag..tag_end], b"raw", 0).is_none() {
        return None;
    }
    let underscore = find(data, b"_", tag_end)?;
    let raw = &data[underscore + 1..];
    let head = &data[..tag];
    let wide = attribute(head, b"<VTKFile", b"header_type").is_some_and(|t| t == b"UInt64");
    let big = attribute(head, b"<VTKFile", b"byte_order").is_some_and(|t| t == b"BigEndian");
    let compressed = attribute(head, b"<VTKFile", b"compressor").is_some_and(|c| !c.is_empty());
    let size = if wide { 8 } else { 4 };
    let number = |at: usize| -> Option<usize> {
        let b = raw.get(at..at + size)?;
        let v = match (wide, big) {
            (true, false) => u64::from_le_bytes(b.try_into().ok()?),
            (true, true) => u64::from_be_bytes(b.try_into().ok()?),
            (false, false) => u32::from_le_bytes(b.try_into().ok()?) as u64,
            (false, true) => u32::from_be_bytes(b.try_into().ok()?) as u64,
        };
        usize::try_from(v).ok()
    };
    // every offset attribute before the appended data: where its number is and what it says
    let mut offsets: Vec<(usize, usize, usize)> = Vec::new();
    let mut at = 0;
    while let Some(k) = find(head, b"offset=\"", at) {
        let start = k + 8;
        let end = find(head, b"\"", start)?;
        // only the attribute itself, not one of a longer name ("component_offset")
        let plain = k == 0 || head[k - 1].is_ascii_whitespace();
        if plain {
            let value = std::str::from_utf8(&head[start..end]).ok()?.trim().parse::<usize>().ok()?;
            offsets.push((start, end, value));
        }
        at = end;
    }
    let mut starts: Vec<usize> = offsets.iter().map(|o| o.2).collect();
    starts.sort_unstable();
    starts.dedup();
    let b64 = base64::engine::general_purpose::STANDARD;
    let mut encoded = String::new();
    let mut moved: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    for &start in &starts {
        moved.insert(start, encoded.len());
        if compressed {
            let blocks = number(start)?;
            let header = size * (3 + blocks);
            let total: usize = (0..blocks).map(|b| number(start + size * (3 + b))).sum::<Option<usize>>()?;
            encoded.push_str(&b64.encode(raw.get(start..start + header)?));
            encoded.push_str(&b64.encode(raw.get(start + header..start + header + total)?));
        } else {
            let bytes = number(start)?;
            encoded.push_str(&b64.encode(raw.get(start..start + size + bytes)?));
        }
    }
    let mut out = Vec::with_capacity(head.len() + encoded.len() + 64);
    let mut last = 0;
    for (start, end, value) in offsets {
        out.extend_from_slice(&head[last..start]);
        out.extend_from_slice(moved.get(&value)?.to_string().as_bytes());
        last = end;
    }
    out.extend_from_slice(&head[last..]);
    out.extend_from_slice(b"<AppendedData encoding=\"base64\">\n_");
    out.extend_from_slice(encoded.as_bytes());
    out.extend_from_slice(b"\n</AppendedData>\n</VTKFile>\n");
    Some(out)
}
