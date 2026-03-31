// SPDX-License-Identifier: GPL-2.0-only

use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::Path;

use cbfstool_rs::{Error, Result};

pub(crate) fn write_new_firmware_image(path: &Path, contents: Vec<u8>) -> Result<()> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(&contents)?;
    file.flush()?;
    file.sync_all()?;
    if let Some(directory) = path.parent() {
        sync_parent_directory(directory)?;
    }
    Ok(())
}

pub(crate) fn rewrite_firmware_image(path: &Path, contents: Vec<u8>) -> Result<()> {
    let directory = path.parent().unwrap_or_else(|| Path::new("."));
    let file_name = path.file_name().ok_or_else(|| Error::InvalidValue {
        what: "firmware image path",
        value: format!("{} has no file name", path.display()),
    })?;
    let symlink_metadata = fs::symlink_metadata(path)?;
    reject_inode_replacing_path(path, &symlink_metadata)?;
    let metadata = fs::metadata(path)?;
    OpenOptions::new().write(true).open(path)?;

    for attempt in 0..100_u32 {
        let temp_name = format!(
            ".{}.tmp.{}.{}",
            file_name.to_string_lossy(),
            std::process::id(),
            attempt
        );
        let temp_path = directory.join(temp_name);
        match OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temp_path)
        {
            Ok(mut file) => {
                let write_result = fs::set_permissions(&temp_path, metadata.permissions())
                    .and_then(|()| file.write_all(&contents))
                    .and_then(|()| file.flush())
                    .and_then(|()| file.sync_all());
                drop(file);
                if let Err(err) = write_result {
                    let _ = fs::remove_file(&temp_path);
                    return Err(err.into());
                }
                if let Err(err) = fs::rename(&temp_path, path) {
                    let _ = fs::remove_file(&temp_path);
                    return Err(err.into());
                }
                sync_parent_directory(directory)?;
                return Ok(());
            }
            Err(err) if err.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(err) => return Err(err.into()),
        }
    }

    Err(Error::InvalidValue {
        what: "temporary firmware image path",
        value: "could not create a unique same-directory temporary file".to_owned(),
    })
}

#[cfg(unix)]
fn reject_inode_replacing_path(path: &Path, metadata: &fs::Metadata) -> Result<()> {
    use std::os::unix::fs::MetadataExt;

    if metadata.file_type().is_symlink() {
        return Err(Error::InvalidValue {
            what: "firmware image path",
            value: format!(
                "{} is a symlink; refusing to replace it atomically",
                path.display()
            ),
        });
    }
    if metadata.nlink() > 1 {
        return Err(Error::InvalidValue {
            what: "firmware image path",
            value: format!(
                "{} has multiple hard links; refusing to replace one link atomically",
                path.display()
            ),
        });
    }
    Ok(())
}

#[cfg(not(unix))]
fn reject_inode_replacing_path(_path: &Path, _metadata: &fs::Metadata) -> Result<()> {
    Ok(())
}

#[cfg(unix)]
fn sync_parent_directory(directory: &Path) -> Result<()> {
    Ok(OpenOptions::new().read(true).open(directory)?.sync_all()?)
}

#[cfg(not(unix))]
fn sync_parent_directory(_directory: &Path) -> Result<()> {
    Ok(())
}
