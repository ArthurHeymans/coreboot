// SPDX-License-Identifier: GPL-2.0-only

//! Hash algorithms and serialized vboot hash values.

use core::fmt;
use core::str::FromStr;

use crate::error::{Error, Result};

/// Identifies a vboot hash algorithm by its `vb2_hash_algorithm` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HashAlgorithm {
    /// SHA-1.
    Sha1 = 1,
    /// SHA-256.
    Sha256 = 2,
    /// SHA-512.
    Sha512 = 3,
    /// SHA-224.
    Sha224 = 4,
    /// SHA-384.
    Sha384 = 5,
}

impl HashAlgorithm {
    /// Creates an algorithm from its numeric vboot ID.
    pub fn from_raw(raw: u8) -> Result<Self> {
        match raw {
            1 => Ok(Self::Sha1),
            2 => Ok(Self::Sha256),
            3 => Ok(Self::Sha512),
            4 => Ok(Self::Sha224),
            5 => Ok(Self::Sha384),
            _ => Err(Error::InvalidValue {
                what: "vboot hash algorithm",
                value: raw.to_string(),
            }),
        }
    }

    /// Returns the numeric vboot algorithm ID.
    pub fn raw(self) -> u8 {
        self as u8
    }

    /// Returns the canonical lower-case algorithm name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Sha1 => "sha1",
            Self::Sha256 => "sha256",
            Self::Sha512 => "sha512",
            Self::Sha224 => "sha224",
            Self::Sha384 => "sha384",
        }
    }

    /// Returns the digest size in bytes.
    pub fn digest_size(self) -> usize {
        match self {
            Self::Sha1 => 20,
            Self::Sha224 => 28,
            Self::Sha256 => 32,
            Self::Sha384 => 48,
            Self::Sha512 => 64,
        }
    }
}

impl FromStr for HashAlgorithm {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        match s.to_ascii_lowercase().replace('-', "").as_str() {
            "sha1" => Ok(Self::Sha1),
            "sha224" => Ok(Self::Sha224),
            "sha256" => Ok(Self::Sha256),
            "sha384" => Ok(Self::Sha384),
            "sha512" => Ok(Self::Sha512),
            _ => {
                let raw = if let Some(hex) = s.strip_prefix("0x") {
                    u8::from_str_radix(hex, 16).ok()
                } else {
                    s.parse::<u8>().ok()
                };
                raw.map_or_else(
                    || {
                        Err(Error::InvalidValue {
                            what: "vboot hash algorithm",
                            value: s.to_owned(),
                        })
                    },
                    Self::from_raw,
                )
            }
        }
    }
}

impl fmt::Display for HashAlgorithm {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Holds a vboot `struct vb2_hash` payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Vb2Hash {
    algorithm: HashAlgorithm,
    digest: Vec<u8>,
}

impl Vb2Hash {
    /// Creates a vboot hash value after validating the digest length.
    pub fn new(algorithm: HashAlgorithm, digest: &[u8]) -> Result<Self> {
        if digest.len() != algorithm.digest_size() {
            return Err(Error::InvalidValue {
                what: "vboot hash digest length",
                value: format!(
                    "{} digest has {} bytes, expected {}",
                    algorithm,
                    digest.len(),
                    algorithm.digest_size()
                ),
            });
        }
        Ok(Self {
            algorithm,
            digest: digest.to_vec(),
        })
    }

    /// Returns the algorithm stored in this hash.
    pub fn algorithm(&self) -> HashAlgorithm {
        self.algorithm
    }

    /// Returns the digest bytes.
    pub fn digest(&self) -> &[u8] {
        &self.digest
    }

    /// Serializes as `struct vb2_hash`: three reserved zero bytes, algorithm, digest.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(4 + self.digest.len());
        bytes.extend_from_slice(&[0, 0, 0, self.algorithm.raw()]);
        bytes.extend_from_slice(&self.digest);
        bytes
    }
}

/// Computes a digest with the selected vboot hash algorithm.
#[cfg(feature = "vboot")]
pub fn digest(algorithm: HashAlgorithm, data: &[u8]) -> Result<Vec<u8>> {
    use sha1::Digest as _;

    let digest = match algorithm {
        HashAlgorithm::Sha1 => sha1::Sha1::digest(data).to_vec(),
        HashAlgorithm::Sha224 => sha2::Sha224::digest(data).to_vec(),
        HashAlgorithm::Sha256 => sha2::Sha256::digest(data).to_vec(),
        HashAlgorithm::Sha384 => sha2::Sha384::digest(data).to_vec(),
        HashAlgorithm::Sha512 => sha2::Sha512::digest(data).to_vec(),
    };
    Ok(digest)
}

/// Reports that digest calculation is unavailable without the `vboot` feature.
#[cfg(not(feature = "vboot"))]
pub fn digest(_algorithm: HashAlgorithm, _data: &[u8]) -> Result<Vec<u8>> {
    Err(Error::InvalidValue {
        what: "vboot hash support",
        value: "the vboot feature is disabled".to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_vboot_hash_names_and_ids() -> Result<()> {
        assert_eq!("sha1".parse::<HashAlgorithm>()?, HashAlgorithm::Sha1);
        assert_eq!("SHA-256".parse::<HashAlgorithm>()?, HashAlgorithm::Sha256);
        assert_eq!("3".parse::<HashAlgorithm>()?, HashAlgorithm::Sha512);
        assert_eq!(HashAlgorithm::Sha384.raw(), 5);
        Ok(())
    }

    #[test]
    fn serializes_vb2_hash_layout() -> Result<()> {
        let digest = [0xa5; 32];
        let hash = Vb2Hash::new(HashAlgorithm::Sha256, &digest)?;
        let bytes = hash.to_bytes();
        assert_eq!(bytes.len(), 36);
        assert_eq!(&bytes[..4], &[0, 0, 0, 2]);
        assert_eq!(&bytes[4..], digest.as_slice());
        Ok(())
    }

    #[test]
    fn rejects_wrong_digest_size() {
        let result = Vb2Hash::new(HashAlgorithm::Sha512, &[0; 32]);
        assert!(matches!(
            result,
            Err(Error::InvalidValue {
                what: "vboot hash digest length",
                ..
            })
        ));
    }

    #[cfg(feature = "vboot")]
    #[test]
    fn computes_sha256_digest() -> Result<()> {
        let digest = digest(HashAlgorithm::Sha256, b"abc")?;
        assert_eq!(
            hex_string(&digest),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        Ok(())
    }

    #[cfg(feature = "vboot")]
    fn hex_string(bytes: &[u8]) -> String {
        let mut output = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            use core::fmt::Write as _;
            let _ = write!(&mut output, "{byte:02x}");
        }
        output
    }
}
