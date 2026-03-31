# cbfstool-rs

Experimental Rust implementation of coreboot's `util/cbfstool`.

This package is intentionally split into:

- `cbfstool_rs`: a Rust library for parsing firmware images, FMAP regions, and
  CBFS entries.
- `cbfstool-rs`: a standalone CLI that follows the C `cbfstool` command shape.

## Source layout

- `src/lib.rs` exports the library API. Library modules avoid CLI-only
  dependencies so users can disable default features for parser/manipulation use.
- `src/bin/cbfstool/main.rs` is the thin CLI entry point and dispatcher.
- `src/bin/cbfstool/cli.rs` contains `clap` argument definitions.
- `src/bin/cbfstool/commands.rs` implements the main CBFS/image subcommands.
- `src/bin/cbfstool/tools/` contains related-tool implementations such as
  `fmap`, `ifwi`, `ifit`, `elog`, CSE helpers, compression, AMD compression,
  and `rmod`.

## Current status

Implemented first slice:

- `layout [-w]`
- `print [-r REGION] [-k]`
- `read [-r REGION] -f FILE`
- `write [-r REGION] -f FILE [-u|-d] [-i BYTE] [-F]`
- `extract [-r REGION] -n NAME -f FILE [-U]`
- `add [-r REGION] -f FILE -n NAME [-t TYPE] [-c none|lzma|lz4|zstd] [-b OFFSET|-a ALIGN]`
- `add-int [-r REGION] -i VALUE -n NAME [-b OFFSET|-a ALIGN]`
- `add-flat-binary [-r REGION] -f FILE -n NAME -l LOAD -e ENTRY [-c none|lzma|lz4|zstd] [-b OFFSET|-a ALIGN]`
- `add-payload [-r REGION] -f ELF -n NAME [-c none|lzma|lz4|zstd] [-b OFFSET|-a ALIGN]`
- `add-stage [-r REGION] -f ELF -n NAME [-S SECTIONS] [-c none|lzma|lz4|zstd] [-b OFFSET|-a ALIGN]`
- `add-master-header [-r REGION] [-j TOPSWAP_SIZE]`
- `create -s SIZE`
- `create -M FMAP [-r REGION[,REGION...]]`
- `remove [-r REGION] -n NAME`
- `compact [-r REGION]`
- `copy [-r REGION] -R SOURCE_REGION`
- `expand [-r REGION]`
- `truncate [-r REGION]`

Implemented related-tool subcommands:

- `fmap [-h HEADER] [-R REGIONS] FMD FMAP`
- `cbfs-compression benchmark`
- `cbfs-compression compress INFILE OUTFILE ALGO`
- `cbfs-compression rawcompress INFILE OUTFILE ALGO`
- `amd-compress -i INFILE -o OUTFILE -c [-m HEX_MAX]`
- `amd-compress -i INFILE -o OUTFILE -u [-m HEX_MAX]`
- `amd-compress -i INFILE -o OUTFILE -p [-m HEX_MAX]`
- `rmod -i INELF -o OUTELF`
- `ifwi FILE print [-d] [-s]`
- `ifwi FILE extract -f FILE -n NAME [-d -e ENTRY] [-s]`
- `ifit -f FILE -r REGION [-D] [-c] [-d NUMBER] [-H OFFSET] [-F -n NAME] [-a|-A -n NAME -t TYPE -s COUNT [-R REGION]]`
- `elog list -f FILE [-U]`
- `elog clear -f FILE`
- `elog add -f FILE TYPE [HEX_DATA]`
- `cse-fpt FILE print [-n NAME]`
- `cse-fpt FILE dump [-o OUTPUT_DIR] [-n NAME]`
- `cse-serger FILE print [-s] [-n NAME] [-t TYPE]`
- `cse-serger FILE dump [-o DIR] [-n NAME] [-t TYPE]`
- `cse-serger FILE create-layout -v VERSION [--dp OFF:SIZE] [--bp1 OFF:SIZE] ...`
- `cse-serger FILE print-layout -v VERSION [--dp OFF:SIZE] [--bp1 OFF:SIZE] ...`
- `cse-serger FILE create-bpdt -v VERSION`
- `build-image --manifest MANIFEST.json [-o OUTPUT]`

The binary uses a `clap` derive-based command line and keeps the existing
`cbfstool FILE COMMAND [OPTIONS]` shape where that is useful. Add-style,
print, write, remove, compact, copy, expand, truncate, and create operations
accept comma-separated FMAP region lists where the matching C tools do. Layout
read-only filtering is currently an advisory display hint; mutating commands do
not yet enforce FMAP read-only policy. Mutating commands rewrite firmware images through a same-directory
temporary file and refuse Unix symlink or multi-hardlink image paths to avoid
silently changing link targets or inode-sharing semantics. Related tools from
`util/cbfstool` are available as top-level subcommands/aliases:
`fmap`/`fmaptool`, `rmod`/`rmodtool`, `ifwi`/`ifwitool`, `ifit`/`ifittool`,
`elog`/`elogtool`, `cse-fpt`/`cse_fpt`, and `cse-serger`/`cse_serger`. Unknown
commands or unsupported options fail during argument parsing before the image
is opened.

The on-flash FMAP/CBFS fixed-size records are represented as `zerocopy` wire
structs with explicit endian integer fields. Higher-level library types keep
owned names and borrowed payload slices, so callers can inspect entries without
copying file data.

The `clap` dependency is behind the default `cli` feature. Compression backends
are behind the default `compression` feature and use Rust-native crates:
`lzma-rs`, `lz4_flex`, and `rust-zstd`. ELF inspection support is behind the
`elf` feature and uses the `object` crate. Library users that only need the
minimal parser can depend on this package with `default-features = false` and
only pull in the core parsing/manipulation library and `zerocopy`.

Extraction currently writes decompressed data for LZMA/LZ4/ZSTD entries, or raw
compressed bytes with `-U`. `create` initializes either a modern headerless CBFS
image from `-s SIZE` or an FMAP-partitioned image from `-M FMAP`, with selected
CBFS regions initialized through `-r`. `add` can create uncompressed or
compressed raw CBFS entries; if compression is
not smaller than the source, it stores the entry uncompressed. Add-style
commands accept `-b/--base-address` as a CBFS-region-relative placement offset
and `-a/--alignment` to choose the first free aligned offset. `add-flat-binary`
and `add-payload` create SELF payload entries, and `add-stage` creates stage
entries with a stage-header attribute and optional whole-stage compression.
`add-master-header` writes a CBFS master header entry and updates the region
tail pointer. `build-image` consumes a versioned JSON manifest that can create
a new FMAP-partitioned or single-CBFS image, add CBFS entries and integer
entries, place a bootblock entry, and write raw FMAP regions in one invocation.
`amd-compress` handles the zlib-with-256-byte-header format used
by AMD firmware payloads, including ELF-to-progbits extraction for compress/copy
inputs when the `elf` feature is enabled. `fmap` compiles the fmd descriptor
language used by fmaptool, including offset/size inference, `CBFS` and
`PRESERVE` annotations, generated C headers, and CBFS region-list output.
`cse-fpt` can print and dump CSE FPT 0x20/0x21 partition tables. `ifwi` can
print/extract BPDT and S-BPDT sub-partitions, `ifit` can dump/clear/delete FIT
entries, set FIT pointers, and add CBFS or raw-region FIT entries, `elog` can
list/clear/add file-backed event logs, and `cse-serger` can print/dump BPDT
sub-partitions plus create BPDT/layout binaries. Image creation and CBFS
attribute serialization are available through library APIs
(`OwnedFirmwareImage::create_empty_cbfs`, `OwnedFirmwareImage::create_partitioned`,
and `AttributeBuilder`) so future CLI parity work can be reused by other Rust
front-ends.

## Manifest image builds

`build-image` is the first step toward replacing the coreboot build's generated
`cbfstool` command stream with a machine-readable artifact. The manifest is JSON
with `version: 1`. It can specify `output`, either an `fmap` object with a binary
FMAP and `cbfs_regions` list or a plain `rom_size`, a `bootblock` entry, `cbfs`
entries, `ints`, and `raw_regions` writes. CBFS entry `type` accepts normal CBFS
types plus `stage`, `payload`, and `flat-binary`; entries may specify
`compression`, `regions`, `placement` (`base` or `align`), `ibb`, `padding`,
and `hash`. Raw region writes support `exact`, `fill-upward`, and `fill-downward`
modes with an optional fill byte. A top-level `vboot` object can expand existing
GBB blobs, FWID strings, shared-data initialization, and explicit
`cbfs_verification` hash attributes for listed CBFS regions. When
`cbfs_verification` is enabled and the manifest `bootblock` source contains a
metadata-hash anchor, `build-image` defers that raw bootblock entry until after
other manifest contents are present, computes CBFS metadata/FMAP hashes from a
provisional image, patches the source bytes, then inserts the patched bootblock.
Finalization also patches anchors already present in `BOOTBLOCK`, `BOOTBLOCK_B`,
CBFS `bootblock`, or CBFS `apu/amdfw` containers, and accepts explicit
`vboot.cbfs_verification.anchor_containers` entries for unusual layouts.

The manifest support intentionally covers the build-system data model first. It
does not yet patch CBFS verification anchors or model FIT operations in the
manifest; those remain follow-up work before the manifest path can become the
default coreboot image assembly path.

## Development

```sh
cargo test --manifest-path util/cbfstool-rs/Cargo.toml
cargo test --manifest-path util/cbfstool-rs/Cargo.toml --no-default-features
cargo run --manifest-path util/cbfstool-rs/Cargo.toml --bin cbfstool-rs -- ROM print
```

## Follow-up parity plan

1. Port remaining attribute workflows as reusable library builders/parsers:
   verified-boot hashes, explicit padding, generated attributes, XIP/FSP
   metadata, IBB markers, topswap metadata, and memory-map metadata.
2. Extend payload/stage builders as library APIs with Linux bzImage/initrd/cmdline
   support, FV/FIT payload input handling, exact PT_LOAD flag mapping, ignored
   sections, XIP stages, and platform fixups.
3. Complete modern CBFS/image workflows: XIP/FSP handling, IBB, mmap metadata,
   additional payload/stage formats, fixups, master-header detection for existing
   images, and top-aligned offset handling where still needed.
4. Add remaining global selectors/policies: explicit `--header-offset`,
   bootblock/flashmap inputs, complete source-region behavior, fill direction,
   force, read-only FMAP write enforcement, and security-section filtering.
5. Complete advanced related-tool parity: IFWI create/add/delete/replace/repack,
   flashrom-backed ELOG, and CSE serger add/create-cse-region workflows.
6. Add command-by-command feature-parity tests against the C `util/cbfstool`.
   The goal is behavioral feature parity for supported workflows, not preserving
   C implementation quirks or bug-for-bug compatibility. Legacy `create -m ARCH`
   is intentionally not planned because no in-tree workflow uses it.
