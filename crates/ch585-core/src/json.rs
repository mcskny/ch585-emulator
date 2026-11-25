use std::fmt::Write as _;

/// Minimal dependency-free JSON AST for deterministic output and configuration loading.
#[derive(Debug, Clone, PartialEq)]
pub enum JsonValue {
    Null,
    Bool(bool),
    Number(f64),
    Str(String),
    Array(Vec<JsonValue>),
    Object(Vec<(String, JsonValue)>),
}

impl JsonValue {
    pub fn as_object(&self) -> Option<&[(String, JsonValue)]> {
        match self {
            Self::Object(pairs) => Some(pairs),
            _ => None,
        }
    }

    pub fn get(&self, key: &str) -> Option<&JsonValue> {
        self.as_object()
            .and_then(|pairs| pairs.iter().rev().find(|(k, _)| k == key).map(|(_, v)| v))
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Self::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Number(n) if n.is_finite() => Some(*n),
            _ => None,
        }
    }

    pub fn as_u64(&self) -> Option<u64> {
        self.as_f64().and_then(|n| {
            if n >= 0.0 && n <= u64::MAX as f64 {
                Some(n.round() as u64)
            } else {
                None
            }
        })
    }

    pub fn as_u32(&self) -> Option<u32> {
        self.as_u64().and_then(|n| u32::try_from(n).ok())
    }

    pub fn as_u8(&self) -> Option<u8> {
        self.as_u64().and_then(|n| u8::try_from(n).ok())
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Self::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub fn as_array(&self) -> Option<&[JsonValue]> {
        match self {
            Self::Array(items) => Some(items),
            _ => None,
        }
    }

    pub fn to_json_string(&self, pretty: bool) -> String {
        let mut out = String::new();
        self.write_to(&mut out, pretty, 0);
        out
    }

    fn write_to(&self, out: &mut String, pretty: bool, indent: usize) {
        match self {
            Self::Null => out.push_str("null"),
            Self::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            Self::Number(n) => {
                if !n.is_finite() {
                    out.push_str("0.0");
                } else if n.fract() == 0.0
                    && *n >= -9_007_199_254_740_991.0
                    && *n <= 9_007_199_254_740_991.0
                {
                    let _ = write!(out, "{}", *n as i64);
                } else {
                    let _ = write!(out, "{n}");
                }
            }
            Self::Str(s) => {
                out.push('"');
                push_escaped_json_str(out, s);
                out.push('"');
            }
            Self::Array(items) => {
                if items.is_empty() {
                    out.push_str("[]");
                    return;
                }
                out.push('[');
                for (i, item) in items.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    if pretty {
                        out.push('\n');
                        push_indent(out, indent + 2);
                    }
                    item.write_to(out, pretty, indent + 2);
                }
                if pretty {
                    out.push('\n');
                    push_indent(out, indent);
                }
                out.push(']');
            }
            Self::Object(pairs) => {
                if pairs.is_empty() {
                    out.push_str("{}");
                    return;
                }
                out.push('{');
                for (i, (k, v)) in pairs.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    if pretty {
                        out.push('\n');
                        push_indent(out, indent + 2);
                    }
                    out.push('"');
                    push_escaped_json_str(out, k);
                    out.push_str(if pretty { "\": " } else { "\":" });
                    v.write_to(out, pretty, indent + 2);
                }
                if pretty {
                    out.push('\n');
                    push_indent(out, indent);
                }
                out.push('}');
            }
        }
    }
}

fn push_indent(out: &mut String, indent: usize) {
    for _ in 0..indent {
        out.push(' ');
    }
}

pub fn push_escaped_json_str(out: &mut String, s: &str) {
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
}

pub fn escape_json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    push_escaped_json_str(&mut out, s);
    out
}

pub fn parse_json(src: &str) -> Result<JsonValue, String> {
    let mut parser = Parser {
        bytes: src.as_bytes(),
        pos: 0,
    };
    parser.skip_ws();
    let val = parser.parse_value()?;
    parser.skip_ws();
    if parser.pos != parser.bytes.len() {
        return Err(format!("Unexpected trailing JSON data at byte {}", parser.pos));
    }
    Ok(val)
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn skip_ws(&mut self) {
        while let Some(b) = self.peek() {
            if matches!(b, b' ' | b'\n' | b'\r' | b'\t') {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    fn expect(&mut self, expected: u8) -> Result<(), String> {
        match self.peek() {
            Some(b) if b == expected => {
                self.pos += 1;
                Ok(())
            }
            Some(b) => Err(format!(
                "Expected '{}' at byte {}, found '{}'",
                expected as char, self.pos, b as char
            )),
            None => Err(format!(
                "Expected '{}' at byte {}, found EOF",
                expected as char, self.pos
            )),
        }
    }

    fn parse_value(&mut self) -> Result<JsonValue, String> {
        self.skip_ws();
        match self.peek() {
            Some(b'n') => self.parse_literal(b"null", JsonValue::Null),
            Some(b't') => self.parse_literal(b"true", JsonValue::Bool(true)),
            Some(b'f') => self.parse_literal(b"false", JsonValue::Bool(false)),
            Some(b'"') => self.parse_string().map(JsonValue::Str),
            Some(b'[') => self.parse_array(),
            Some(b'{') => self.parse_object(),
            Some(b'-' | b'0'..=b'9') => self.parse_number(),
            Some(b) => Err(format!(
                "Unexpected character '{}' at byte {}",
                b as char, self.pos
            )),
            None => Err("Unexpected EOF while parsing JSON".to_string()),
        }
    }

    fn parse_literal(&mut self, lit: &[u8], val: JsonValue) -> Result<JsonValue, String> {
        if self.bytes.get(self.pos..self.pos + lit.len()) == Some(lit) {
            self.pos += lit.len();
            Ok(val)
        } else {
            Err(format!("Invalid JSON literal at byte {}", self.pos))
        }
    }

    fn parse_string(&mut self) -> Result<String, String> {
        self.expect(b'"')?;
        let mut out = String::new();
        while let Some(b) = self.peek() {
            if b == b'"' {
                self.pos += 1;
                return Ok(out);
            }
            if b == b'\\' {
                self.pos += 1;
                let esc = self
                    .peek()
                    .ok_or_else(|| "Unterminated escape in JSON string".to_string())?;
                self.pos += 1;
                match esc {
                    b'"' => out.push('"'),
                    b'\\' => out.push('\\'),
                    b'/' => out.push('/'),
                    b'b' => out.push('\u{0008}'),
                    b'f' => out.push('\u{000C}'),
                    b'n' => out.push('\n'),
                    b'r' => out.push('\r'),
                    b't' => out.push('\t'),
                    b'u' => {
                        let code = self.parse_hex4()?;
                        if (0xD800..=0xDBFF).contains(&code)
                            && self.bytes.get(self.pos..self.pos + 2) == Some(b"\\u")
                        {
                            self.pos += 2;
                            let low = self.parse_hex4()?;
                            if (0xDC00..=0xDFFF).contains(&low) {
                                let cp = 0x10000
                                    + (((code as u32 - 0xD800) << 10) | (low as u32 - 0xDC00));
                                if let Some(ch) = char::from_u32(cp) {
                                    out.push(ch);
                                    continue;
                                }
                            }
                            return Err("Invalid UTF-16 surrogate pair in JSON".to_string());
                        }
                        let ch = char::from_u32(code as u32)
                            .ok_or_else(|| "Invalid \\uXXXX code point in JSON".to_string())?;
                        out.push(ch);
                    }
                    other => {
                        return Err(format!(
                            "Invalid escape '\\{}' in JSON string",
                            other as char
                        ));
                    }
                }
                continue;
            }
            let start = self.pos;
            while let Some(&c) = self.bytes.get(self.pos) {
                if c == b'"' || c == b'\\' {
                    break;
                }
                if c < 0x20 {
                    return Err(format!("Unescaped control char in JSON at byte {}", self.pos));
                }
                self.pos += 1;
            }
            let slice = std::str::from_utf8(&self.bytes[start..self.pos])
                .map_err(|e| format!("Invalid UTF-8 in JSON string: {e}"))?;
            out.push_str(slice);
        }
        Err("Unterminated JSON string".to_string())
    }

    fn parse_hex4(&mut self) -> Result<u16, String> {
        let slice = self
            .bytes
            .get(self.pos..self.pos + 4)
            .ok_or_else(|| "Truncated \\uXXXX escape".to_string())?;
        let s = std::str::from_utf8(slice).map_err(|_| "Invalid hex in \\uXXXX".to_string())?;
        let v = u16::from_str_radix(s, 16)
            .map_err(|_| format!("Invalid hex digits '{s}' in \\uXXXX"))?;
        self.pos += 4;
        Ok(v)
    }

    fn parse_number(&mut self) -> Result<JsonValue, String> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.pos += 1;
        }
        match self.peek() {
            Some(b'0') => self.pos += 1,
            Some(b'1'..=b'9') => {
                while self.peek().is_some_and(|b| b.is_ascii_digit()) {
                    self.pos += 1;
                }
            }
            _ => return Err(format!("Invalid JSON number at byte {start}")),
        }
        if self.peek() == Some(b'.') {
            self.pos += 1;
            if !self.peek().is_some_and(|b| b.is_ascii_digit()) {
                return Err(format!("Invalid JSON fraction at byte {}", self.pos));
            }
            while self.peek().is_some_and(|b| b.is_ascii_digit()) {
                self.pos += 1;
            }
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.pos += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            if !self.peek().is_some_and(|b| b.is_ascii_digit()) {
                return Err(format!("Invalid JSON exponent at byte {}", self.pos));
            }
            while self.peek().is_some_and(|b| b.is_ascii_digit()) {
                self.pos += 1;
            }
        }
        let s = std::str::from_utf8(&self.bytes[start..self.pos])
            .map_err(|_| "Invalid UTF-8 in number".to_string())?;
        let n: f64 = s
            .parse()
            .map_err(|_| format!("Failed to parse JSON number '{s}'"))?;
        if !n.is_finite() {
            return Err(format!("Non-finite JSON number '{s}'"));
        }
        Ok(JsonValue::Number(n))
    }

    fn parse_array(&mut self) -> Result<JsonValue, String> {
        self.expect(b'[')?;
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.pos += 1;
            return Ok(JsonValue::Array(items));
        }
        loop {
            items.push(self.parse_value()?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.pos += 1,
                Some(b']') => {
                    self.pos += 1;
                    return Ok(JsonValue::Array(items));
                }
                _ => return Err(format!("Expected ',' or ']' in array at byte {}", self.pos)),
            }
        }
    }

    fn parse_object(&mut self) -> Result<JsonValue, String> {
        self.expect(b'{')?;
        let mut pairs = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(JsonValue::Object(pairs));
        }
        loop {
            self.skip_ws();
            let key = self.parse_string()?;
            self.skip_ws();
            self.expect(b':')?;
            let val = self.parse_value()?;
            pairs.push((key, val));
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.pos += 1,
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(JsonValue::Object(pairs));
                }
                _ => return Err(format!("Expected ',' or '}}' in object at byte {}", self.pos)),
            }
        }
    }
}
