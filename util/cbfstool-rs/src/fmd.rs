// SPDX-License-Identifier: GPL-2.0-only

use std::collections::HashSet;

use crate::error::{Error, Result};
use crate::format::fmap::{FMAP_AREA_PRESERVE, FMAP_SIGNATURE, FMAP_STRLEN};

const FMAP_HEADER_LEN: usize = 56;
const FMAP_AREA_LEN: usize = 42;

#[derive(Debug, Clone)]
pub struct FlashmapDescriptor {
    name: String,
    offset: Option<u32>,
    size: Option<u32>,
    cbfs: bool,
    preserve: bool,
    children: Vec<FlashmapDescriptor>,
}

impl FlashmapDescriptor {
    pub fn parse(input: &str) -> Result<Self> {
        let mut parser = Parser::new(input)?;
        let mut root = parser.parse_root()?;
        parser.expect_end()?;
        root.validate_and_complete()?;
        Ok(root)
    }

    pub fn to_fmap_bytes(&self) -> Result<Vec<u8>> {
        let size = self.size.ok_or(Error::InvalidValue {
            what: "FMD root size",
            value: "missing".to_owned(),
        })?;
        if self.name.len() >= FMAP_STRLEN {
            return Err(Error::InvalidValue {
                what: "FMAP image name",
                value: self.name.clone(),
            });
        }

        let mut areas = Vec::new();
        self.collect_areas(0, &mut areas)?;
        let nareas = u16::try_from(areas.len()).map_err(|_| Error::InvalidValue {
            what: "FMAP area count",
            value: areas.len().to_string(),
        })?;
        let mut bytes = vec![0; FMAP_HEADER_LEN + areas.len() * FMAP_AREA_LEN];
        bytes[0..8].copy_from_slice(FMAP_SIGNATURE);
        bytes[8] = 1;
        bytes[9] = 1;
        bytes[10..18].copy_from_slice(&u64::from(self.offset.unwrap_or(0)).to_le_bytes());
        bytes[18..22].copy_from_slice(&size.to_le_bytes());
        write_name(&mut bytes[22..54], &self.name);
        bytes[54..56].copy_from_slice(&nareas.to_le_bytes());

        for (index, area) in areas.iter().enumerate() {
            let start = FMAP_HEADER_LEN + index * FMAP_AREA_LEN;
            bytes[start..start + 4].copy_from_slice(&area.offset.to_le_bytes());
            bytes[start + 4..start + 8].copy_from_slice(&area.size.to_le_bytes());
            write_name(&mut bytes[start + 8..start + 40], &area.name);
            bytes[start + 40..start + 42].copy_from_slice(&area.flags.to_le_bytes());
        }
        Ok(bytes)
    }

    pub fn fmap_size(&self) -> Result<usize> {
        Ok(self.to_fmap_bytes()?.len())
    }

    pub fn cbfs_section_names(&self) -> Vec<String> {
        let mut primary = Vec::new();
        let mut rest = Vec::new();
        self.collect_cbfs_sections(&mut primary, &mut rest);
        primary.extend(rest);
        primary
    }

    pub fn terminal_section_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        self.collect_terminal_sections(&mut names);
        names
    }

    pub fn section_defines(&self) -> Vec<(String, u32, u32)> {
        let mut defines = Vec::new();
        self.collect_section_defines(0, &mut defines);
        defines
    }

    pub fn absolute_offset(&self, name: &str) -> Option<u32> {
        self.find_absolute_offset(name, 0)
    }

    fn validate_and_complete(&mut self) -> Result<()> {
        let mut names = HashSet::new();
        self.validate_names_and_flags(&mut names)?;
        if !names.contains("FMAP") {
            return Err(Error::InvalidValue {
                what: "FMD descriptor",
                value: "missing FMAP section".to_owned(),
            });
        }
        if !self
            .cbfs_section_names()
            .iter()
            .any(|name| name == "COREBOOT")
        {
            return Err(Error::InvalidValue {
                what: "FMD descriptor",
                value: "missing COREBOOT(CBFS) section".to_owned(),
            });
        }
        self.complete_children()
    }

    fn validate_names_and_flags(&self, names: &mut HashSet<String>) -> Result<()> {
        if !names.insert(self.name.clone()) {
            return Err(Error::InvalidValue {
                what: "FMD section",
                value: format!("duplicate name {}", self.name),
            });
        }
        if self.size == Some(0) {
            return Err(Error::InvalidValue {
                what: "FMD section",
                value: format!("{} has zero size", self.name),
            });
        }
        if self.cbfs && !self.children.is_empty() {
            return Err(Error::InvalidValue {
                what: "FMD section",
                value: format!("{} has CBFS flag and children", self.name),
            });
        }
        self.children
            .iter()
            .try_for_each(|child| child.validate_names_and_flags(names))
    }

    fn complete_children(&mut self) -> Result<()> {
        let parent_size = self.size.ok_or(Error::InvalidValue {
            what: "FMD section",
            value: format!("{} has unknown size", self.name),
        })?;
        complete_sibling_layout(&mut self.children, parent_size)?;
        self.children
            .iter_mut()
            .try_for_each(Self::complete_children)
    }

    fn collect_areas(&self, base: u32, areas: &mut Vec<FmapAreaRecord>) -> Result<()> {
        for child in &self.children {
            let offset = child.offset.ok_or(Error::InvalidValue {
                what: "FMD section offset",
                value: child.name.clone(),
            })?;
            let size = child.size.ok_or(Error::InvalidValue {
                what: "FMD section size",
                value: child.name.clone(),
            })?;
            let absolute = base.checked_add(offset).ok_or(Error::InvalidOffset {
                what: "FMD section",
                offset: base as usize,
            })?;
            if child.name.len() >= FMAP_STRLEN {
                return Err(Error::InvalidValue {
                    what: "FMAP section name",
                    value: child.name.clone(),
                });
            }
            let flags = if child.preserve {
                FMAP_AREA_PRESERVE
            } else {
                0
            };
            areas.push(FmapAreaRecord {
                offset: absolute,
                size,
                name: child.name.clone(),
                flags,
            });
            child.collect_areas(absolute, areas)?;
        }
        Ok(())
    }

    fn collect_cbfs_sections(&self, primary: &mut Vec<String>, rest: &mut Vec<String>) {
        if self.cbfs {
            if self.name == "COREBOOT" {
                primary.push(self.name.clone());
            } else {
                rest.push(self.name.clone());
            }
        }
        self.children
            .iter()
            .for_each(|child| child.collect_cbfs_sections(primary, rest));
    }

    fn collect_terminal_sections(&self, names: &mut Vec<String>) {
        if self.children.is_empty() {
            names.push(self.name.clone());
        } else {
            self.children
                .iter()
                .for_each(|child| child.collect_terminal_sections(names));
        }
    }

    fn collect_section_defines(&self, base: u32, defines: &mut Vec<(String, u32, u32)>) {
        for child in &self.children {
            let offset = base + child.offset.unwrap_or(0);
            if let Some(size) = child.size {
                defines.push((child.name.clone(), offset, size));
            }
            child.collect_section_defines(offset, defines);
        }
    }

    fn find_absolute_offset(&self, name: &str, base: u32) -> Option<u32> {
        for child in &self.children {
            let offset = base + child.offset?;
            if child.name == name {
                return Some(offset);
            }
            if let Some(found) = child.find_absolute_offset(name, offset) {
                return Some(found);
            }
        }
        None
    }
}

#[derive(Debug, Clone)]
struct FmapAreaRecord {
    offset: u32,
    size: u32,
    name: String,
    flags: u16,
}

fn complete_sibling_layout(children: &mut [FlashmapDescriptor], parent_size: u32) -> Result<()> {
    let mut index = 0;
    let mut watermark = 0;
    while index < children.len() {
        if children[index].offset.is_none() {
            children[index].offset = Some(watermark);
        }
        if children[index].size.is_some() {
            let child_end = checked_section_end(&children[index])?;
            if child_end > parent_size {
                return Err(section_too_big(&children[index]));
            }
            watermark = child_end;
            index += 1;
            continue;
        }

        let start = children[index].offset.ok_or(Error::InvalidValue {
            what: "FMD section offset",
            value: children[index].name.clone(),
        })?;
        let mut end_index = index + 1;
        while end_index < children.len() && children[end_index].offset.is_none() {
            if children[end_index].size.is_none() {
                return Err(Error::InvalidValue {
                    what: "FMD section",
                    value: format!(
                        "cannot determine either offset or size of section {}",
                        children[end_index].name
                    ),
                });
            }
            end_index += 1;
        }
        let end = if end_index < children.len() {
            children[end_index].offset.ok_or(Error::InvalidValue {
                what: "FMD section offset",
                value: children[end_index].name.clone(),
            })?
        } else {
            parent_size
        };
        if end < start {
            return Err(section_too_big(&children[index]));
        }
        let mut cursor = end;
        for child in children[index..end_index].iter_mut().rev() {
            if child.size.is_none() {
                child.size = Some(
                    cursor
                        .checked_sub(child.offset.unwrap_or(start))
                        .ok_or_else(|| section_too_big(child))?,
                );
                cursor = child.offset.unwrap_or(start);
            } else if let Some(size) = child.size {
                cursor = cursor
                    .checked_sub(size)
                    .ok_or_else(|| section_too_big(child))?;
                child.offset = Some(cursor);
            }
        }
        watermark = end;
        index = end_index;
    }
    Ok(())
}

fn checked_section_end(section: &FlashmapDescriptor) -> Result<u32> {
    section
        .offset
        .and_then(|offset| section.size.and_then(|size| offset.checked_add(size)))
        .ok_or(Error::InvalidValue {
            what: "FMD section",
            value: format!("{} has invalid layout", section.name),
        })
}

fn section_too_big(section: &FlashmapDescriptor) -> Error {
    Error::InvalidValue {
        what: "FMD section",
        value: format!("{} does not fit", section.name),
    }
}

fn write_name(destination: &mut [u8], name: &str) {
    destination.fill(0);
    destination[..name.len()].copy_from_slice(name.as_bytes());
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum Token {
    Name(String),
    Integer(u32),
    At,
    LBrace,
    RBrace,
    LParen,
    RParen,
}

struct Parser {
    tokens: Vec<Token>,
    position: usize,
}

impl Parser {
    fn new(input: &str) -> Result<Self> {
        Ok(Self {
            tokens: tokenize(input)?,
            position: 0,
        })
    }

    fn parse_root(&mut self) -> Result<FlashmapDescriptor> {
        let name = self.parse_name()?;
        let offset = self.parse_offset()?;
        let size = Some(self.parse_integer()?);
        let children = self.parse_children()?;
        Ok(FlashmapDescriptor {
            name,
            offset,
            size,
            cbfs: false,
            preserve: false,
            children,
        })
    }

    fn parse_region(&mut self) -> Result<FlashmapDescriptor> {
        let name = self.parse_name()?;
        let (cbfs, preserve) = self.parse_flags()?;
        let offset = self.parse_offset()?;
        let size = self.parse_optional_integer()?;
        let children = if self.peek() == Some(&Token::LBrace) {
            self.parse_children()?
        } else {
            Vec::new()
        };
        Ok(FlashmapDescriptor {
            name,
            offset,
            size,
            cbfs,
            preserve,
            children,
        })
    }

    fn parse_children(&mut self) -> Result<Vec<FlashmapDescriptor>> {
        self.expect(Token::LBrace)?;
        let mut children = Vec::new();
        while self.peek() != Some(&Token::RBrace) {
            if self.peek().is_none() {
                return Err(Error::InvalidValue {
                    what: "FMD descriptor",
                    value: "unterminated section list".to_owned(),
                });
            }
            children.push(self.parse_region()?);
        }
        self.expect(Token::RBrace)?;
        Ok(children)
    }

    fn parse_flags(&mut self) -> Result<(bool, bool)> {
        if self.peek() != Some(&Token::LParen) {
            return Ok((false, false));
        }
        self.expect(Token::LParen)?;
        let mut cbfs = false;
        let mut preserve = false;
        while self.peek() != Some(&Token::RParen) {
            match self.next() {
                Some(Token::Name(flag)) if flag == "CBFS" => cbfs = true,
                Some(Token::Name(flag)) if flag == "PRESERVE" => preserve = true,
                Some(token) => {
                    return Err(Error::InvalidValue {
                        what: "FMD flag",
                        value: format!("unexpected {token:?}"),
                    });
                }
                None => {
                    return Err(Error::InvalidValue {
                        what: "FMD flags",
                        value: "unterminated flag list".to_owned(),
                    });
                }
            }
        }
        self.expect(Token::RParen)?;
        Ok((cbfs, preserve))
    }

    fn parse_offset(&mut self) -> Result<Option<u32>> {
        if self.peek() != Some(&Token::At) {
            return Ok(None);
        }
        self.expect(Token::At)?;
        Ok(Some(self.parse_integer()?))
    }

    fn parse_optional_integer(&mut self) -> Result<Option<u32>> {
        match self.peek() {
            Some(Token::Integer(_)) => self.parse_integer().map(Some),
            _ => Ok(None),
        }
    }

    fn parse_integer(&mut self) -> Result<u32> {
        match self.next() {
            Some(Token::Integer(value)) => Ok(value),
            other => Err(Error::InvalidValue {
                what: "FMD integer",
                value: format!("expected integer, got {other:?}"),
            }),
        }
    }

    fn parse_name(&mut self) -> Result<String> {
        match self.next() {
            Some(Token::Name(value)) => Ok(value),
            other => Err(Error::InvalidValue {
                what: "FMD name",
                value: format!("expected name, got {other:?}"),
            }),
        }
    }

    fn expect(&mut self, token: Token) -> Result<()> {
        let actual = self.next();
        if actual == Some(token.clone()) {
            Ok(())
        } else {
            Err(Error::InvalidValue {
                what: "FMD token",
                value: format!("expected {token:?}, got {actual:?}"),
            })
        }
    }

    fn expect_end(&self) -> Result<()> {
        if self.position == self.tokens.len() {
            Ok(())
        } else {
            Err(Error::InvalidValue {
                what: "FMD token",
                value: format!("unexpected {:?}", self.tokens[self.position]),
            })
        }
    }

    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.position)
    }

    fn next(&mut self) -> Option<Token> {
        let token = self.tokens.get(self.position).cloned();
        self.position += usize::from(token.is_some());
        token
    }
}

fn tokenize(input: &str) -> Result<Vec<Token>> {
    let mut tokens = Vec::new();
    for line in input.lines() {
        let line = line.split_once('#').map_or(line, |(before, _)| before);
        let mut chars = line.chars().peekable();
        while let Some(ch) = chars.peek().copied() {
            if ch.is_whitespace() {
                chars.next();
                continue;
            }
            match ch {
                '@' => {
                    chars.next();
                    tokens.push(Token::At);
                }
                '{' => {
                    chars.next();
                    tokens.push(Token::LBrace);
                }
                '}' => {
                    chars.next();
                    tokens.push(Token::RBrace);
                }
                '(' => {
                    chars.next();
                    tokens.push(Token::LParen);
                }
                ')' => {
                    chars.next();
                    tokens.push(Token::RParen);
                }
                _ => {
                    let mut text = String::new();
                    while let Some(next) = chars.peek().copied() {
                        if next.is_whitespace() || matches!(next, '#' | '@' | '{' | '}' | '(' | ')')
                        {
                            break;
                        }
                        text.push(next);
                        chars.next();
                    }
                    if let Some(value) = parse_fmd_integer(&text)? {
                        tokens.push(Token::Integer(value));
                    } else {
                        tokens.push(Token::Name(text));
                    }
                }
            }
        }
    }
    Ok(tokens)
}

fn parse_fmd_integer(text: &str) -> Result<Option<u32>> {
    let (digits, multiplier) = match text.as_bytes().last().copied() {
        Some(b'K' | b'M' | b'G') => (&text[..text.len() - 1], text.as_bytes().last().copied()),
        _ => (text, None),
    };
    let Some(first) = digits.as_bytes().first().copied() else {
        return Ok(None);
    };
    if !first.is_ascii_digit() {
        return Ok(None);
    }
    let (number_text, radix) = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        (hex, 16)
    } else if digits.len() > 1 && digits.starts_with('0') {
        return Err(Error::InvalidValue {
            what: "FMD integer",
            value: format!("octal-like integer {text} is not supported"),
        });
    } else {
        (digits, 10)
    };
    let mut value = u32::from_str_radix(number_text, radix).map_err(|err| Error::InvalidValue {
        what: "FMD integer",
        value: err.to_string(),
    })?;
    value = match multiplier {
        Some(b'K') => value.saturating_mul(1024),
        Some(b'M') => value.saturating_mul(1024 * 1024),
        Some(b'G') => value.saturating_mul(1024 * 1024 * 1024),
        _ => value,
    };
    Ok(Some(value))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_and_emits_fmap() -> Result<()> {
        let fmd = FlashmapDescriptor::parse(
            "FLASH 1M { FMAP 256K COREBOOT(CBFS) 512K RW(PRESERVE) { RW_A 128K RW_B 128K } }",
        )?;

        assert_eq!(fmd.cbfs_section_names(), ["COREBOOT"]);
        assert_eq!(fmd.absolute_offset("FMAP"), Some(0));
        assert_eq!(fmd.absolute_offset("RW_B"), Some(0xe_0000));
        let bytes = fmd.to_fmap_bytes()?;
        assert_eq!(&bytes[..8], FMAP_SIGNATURE);
        assert_eq!(bytes.len(), FMAP_HEADER_LEN + 5 * FMAP_AREA_LEN);
        Ok(())
    }
}
