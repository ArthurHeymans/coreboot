// SPDX-License-Identifier: GPL-2.0-only

use std::path::PathBuf;

use cbfstool_rs::{CbfsType, Compression, HashAlgorithm};
use clap::{ArgAction, ArgGroup, Args, Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(
    name = "cbfstool-rs",
    version,
    about = "Rust CBFS utility",
    long_about = "Experimental Rust implementation of coreboot's util/cbfstool.\n\n\
                  The command shape intentionally follows cbfstool: cbfstool-rs FILE COMMAND [OPTIONS]."
)]
pub(crate) struct Cli {
    /// Firmware image to inspect or modify.
    #[arg(value_name = "FILE")]
    pub(crate) image: PathBuf,

    /// Increase diagnostic verbosity. Accepted for cbfstool command-line parity.
    #[arg(short = 'v', long = "verbose", action = ArgAction::Count, global = true)]
    pub(crate) verbose: u8,

    #[command(subcommand)]
    pub(crate) command: Command,
}

#[derive(Debug, Subcommand)]
pub(crate) enum Command {
    /// Show FMAP regions that cbfstool-rs can access.
    Layout(LayoutArgs),
    /// Print CBFS entries.
    Print(PrintArgs),
    /// Copy an FMAP region to a file.
    Read(ReadArgs),
    /// Extract a CBFS entry to a file.
    Extract(ExtractArgs),
    /// Add a CBFS entry from a file.
    Add(AddArgs),
    /// Add an integer as an 8-byte little-endian raw CBFS entry.
    AddInt(AddIntArgs),
    /// Add a flat binary as a SELF payload.
    #[command(name = "add-flat-binary")]
    AddFlatBinary(AddFlatBinaryArgs),
    /// Add an ELF executable as a SELF payload.
    #[command(name = "add-payload")]
    AddPayload(AddPayloadArgs),
    /// Add an ELF executable as a stage.
    #[command(name = "add-stage")]
    AddStage(AddStageArgs),
    /// Add a CBFS master header entry and update the tail pointer.
    #[command(name = "add-master-header")]
    AddMasterHeader(AddMasterHeaderArgs),
    /// Create an empty CBFS image.
    Create(CreateArgs),
    /// Write raw data into an FMAP region.
    Write(WriteArgs),
    /// Remove a CBFS entry.
    Remove(RemoveArgs),
    /// Compact a CBFS region by moving free space to the end.
    Compact(RegionCommandArgs),
    /// Copy non-empty CBFS entries from one region into another.
    Copy(CopyArgs),
    /// Expand CBFS free space to the end of the selected region.
    Expand(RegionCommandArgs),
    /// Remove trailing CBFS free space and print the effective size.
    Truncate(RegionCommandArgs),
}

#[derive(Debug, Parser)]
#[command(
    name = "cbfstool-rs",
    version,
    about = "Rust firmware utility collection"
)]
pub(crate) struct ToolCli {
    #[command(subcommand)]
    pub(crate) command: ToolCommand,
}

#[derive(Debug, Subcommand)]
pub(crate) enum ToolCommand {
    /// Build a complete firmware image from a machine-readable manifest.
    #[command(name = "build-image")]
    BuildImage(BuildImageArgs),
    /// Compile Flashmap descriptor (fmd) files.
    #[command(alias = "fmaptool")]
    Fmap(FmapToolArgs),
    /// Create relocatable modules.
    #[command(alias = "rmodtool")]
    Rmod(RmodArgs),
    /// Manipulate Intel Firmware Images (IFWI).
    #[command(alias = "ifwitool")]
    Ifwi(IfwiArgs),
    /// Manipulate Intel Firmware Interface Tables (FIT).
    #[command(alias = "ifittool")]
    Ifit(IfitArgs),
    /// Benchmark or inspect CBFS compression algorithms.
    #[command(name = "cbfs-compression", alias = "cbfs-compression-tool")]
    CbfsCompression(CbfsCompressionArgs),
    /// Extract or create AMD zlib-compressed BIOS binaries.
    #[command(name = "amd-compress", alias = "amdcompress")]
    AmdCompress(AmdCompressArgs),
    /// Display ELOG events.
    #[command(alias = "elogtool")]
    Elog(ElogArgs),
    /// Manage Intel CSE Flash Partition Tables (FPT).
    #[command(name = "cse-fpt", alias = "cse_fpt")]
    CseFpt(CseFptArgs),
    /// Stitch Intel CSE components.
    #[command(name = "cse-serger", alias = "cse_serger")]
    CseSerger(CseSergerArgs),
}

impl ToolCommand {
    pub(crate) fn is_tool_name(name: &str) -> bool {
        matches!(
            name,
            "build-image"
                | "fmap"
                | "fmaptool"
                | "rmod"
                | "rmodtool"
                | "ifwi"
                | "ifwitool"
                | "ifit"
                | "ifittool"
                | "cbfs-compression"
                | "cbfs-compression-tool"
                | "amd-compress"
                | "amdcompress"
                | "elog"
                | "elogtool"
                | "cse-fpt"
                | "cse_fpt"
                | "cse-serger"
                | "cse_serger"
        )
    }
}

#[derive(Debug, Args)]
pub(crate) struct BuildImageArgs {
    /// JSON manifest describing the image to build.
    #[arg(short = 'm', long = "manifest", value_name = "FILE")]
    pub(crate) manifest: PathBuf,

    /// Override the output path from the manifest.
    #[arg(short = 'o', long = "output", value_name = "FILE")]
    pub(crate) output: Option<PathBuf>,
}

#[derive(Debug, Args)]
pub(crate) struct CbfsCompressionArgs {
    #[command(subcommand)]
    pub(crate) command: CbfsCompressionCommand,
}

#[derive(Debug, Subcommand)]
pub(crate) enum CbfsCompressionCommand {
    /// Run a simple compression benchmark for all implemented algorithms.
    Benchmark,
    /// Compress a file and prepend the cbfs-compression-tool header.
    Compress {
        #[arg(value_name = "INFILE")]
        input: PathBuf,
        #[arg(value_name = "OUTFILE")]
        output: PathBuf,
        #[arg(value_name = "ALGO")]
        algorithm: Compression,
    },
    /// Compress a file without prepending the cbfs-compression-tool header.
    Rawcompress {
        #[arg(value_name = "INFILE")]
        input: PathBuf,
        #[arg(value_name = "OUTFILE")]
        output: PathBuf,
        #[arg(value_name = "ALGO")]
        algorithm: Compression,
    },
}

#[derive(Debug, Args)]
#[command(disable_help_flag = true)]
pub(crate) struct FmapToolArgs {
    /// Print help.
    #[arg(long = "help", action = ArgAction::Help)]
    pub(crate) help: Option<bool>,

    /// Also produce a C header with FMAP offsets and sections.
    #[arg(short = 'h', value_name = "HEADER")]
    pub(crate) header: Option<PathBuf>,

    /// Also produce a text file listing CBFS regions, comma separated.
    #[arg(short = 'R', value_name = "REGIONS")]
    pub(crate) regions: Option<PathBuf>,

    /// FMD input file, or '-' for stdin.
    #[arg(value_name = "FMD")]
    pub(crate) input: PathBuf,

    /// Binary FMAP output file.
    #[arg(value_name = "FMAP")]
    pub(crate) output: PathBuf,
}

#[derive(Debug, Args)]
#[command(group(ArgGroup::new("mode").required(true).args(["compress", "uncompress", "elfcopy"])))]
pub(crate) struct AmdCompressArgs {
    /// Input file.
    #[arg(short = 'i', long = "infile", value_name = "FILE")]
    pub(crate) input: PathBuf,

    /// Output file.
    #[arg(short = 'o', long = "outfile", value_name = "FILE")]
    pub(crate) output: PathBuf,

    /// Compress input with zlib and prepend the 256-byte AMD header.
    #[arg(short = 'c', long = "compress")]
    pub(crate) compress: bool,

    /// Uncompress input with a 256-byte AMD zlib header.
    #[arg(short = 'u', long = "uncompress")]
    pub(crate) uncompress: bool,

    /// Copy input after ELF-to-progbits conversion, if the input is ELF.
    #[arg(short = 'p', long = "elfcopy")]
    pub(crate) elfcopy: bool,

    /// Maximum uncompressed size as hexadecimal.
    #[arg(short = 'm', long = "maxsize", value_name = "HEX_VAL", value_parser = parse_hex_usize)]
    pub(crate) max_size: Option<usize>,
}

#[derive(Debug, Args)]
pub(crate) struct RmodArgs {
    /// Input executable ELF.
    #[arg(short = 'i', long = "inelf", value_name = "FILE")]
    pub(crate) input: PathBuf,

    /// Output rmodule image.
    #[arg(short = 'o', long = "outelf", value_name = "FILE")]
    pub(crate) output: PathBuf,

    /// Increase diagnostic verbosity. Accepted for rmodtool parity.
    #[arg(short = 'v', long = "verbose", action = ArgAction::Count)]
    pub(crate) verbose: u8,
}

#[derive(Debug, Args)]
pub(crate) struct IfwiArgs {
    /// IFWI image.
    #[arg(value_name = "FILE")]
    pub(crate) image: PathBuf,

    #[command(subcommand)]
    pub(crate) command: IfwiCommand,
}

#[derive(Debug, Subcommand)]
pub(crate) enum IfwiCommand {
    /// Print BPDT and S-BPDT entries.
    Print(IfwiPrintArgs),
    /// Extract a sub-partition or a directory entry.
    Extract(IfwiExtractArgs),
}

#[derive(Debug, Args)]
pub(crate) struct IfwiPrintArgs {
    /// Print contained sub-partition directory entries.
    #[arg(short = 'd', long = "dir-ops", alias = "dir_ops")]
    pub(crate) directory: bool,

    /// Use the second logical boot partition.
    #[arg(short = 's', long = "second-lbp", alias = "second_lbp")]
    pub(crate) second_lbp: bool,
}

#[derive(Debug, Args)]
pub(crate) struct IfwiExtractArgs {
    /// Destination file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// Sub-partition name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,

    /// Extract a directory entry from the sub-partition.
    #[arg(short = 'd', long = "dir-ops", alias = "dir_ops")]
    pub(crate) directory: bool,

    /// Directory entry name to extract.
    #[arg(
        short = 'e',
        long = "subpart-dentry",
        alias = "subpart_dentry",
        value_name = "ENTRY"
    )]
    pub(crate) entry: Option<String>,

    /// Use the second logical boot partition.
    #[arg(short = 's', long = "second-lbp", alias = "second_lbp")]
    pub(crate) second_lbp: bool,
}

#[derive(Debug, Args)]
pub(crate) struct IfitArgs {
    /// File containing the CBFS/FMAP image.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// FMAP region containing the FIT table.
    #[arg(short = 'r', long = "region", value_name = "REGION")]
    pub(crate) region: String,

    /// Dump FIT table after parsing.
    #[arg(short = 'D', long = "dump")]
    pub(crate) dump: bool,

    /// Clear all entries after the FIT header.
    #[arg(short = 'c', long = "clear-table")]
    pub(crate) clear_table: bool,

    /// Delete an existing FIT entry by one-based number.
    #[arg(short = 'd', long = "del-entry", value_name = "NUMBER")]
    pub(crate) delete: Option<usize>,

    /// Do not search for the header; use this offset within the selected region.
    #[arg(short = 'H', long = "header-offset", value_name = "OFFSET", value_parser = parse_usize)]
    pub(crate) header_offset: Option<usize>,

    /// Add a CBFS file as a new entry to FIT.
    #[arg(short = 'a', long = "add-cbfs-entry")]
    pub(crate) add_cbfs_entry: bool,

    /// Add a raw FMAP region as a new entry to FIT.
    #[arg(short = 'A', long = "add-region")]
    pub(crate) add_region: bool,

    /// Set the reset-vector FIT pointer to a CBFS FIT table.
    #[arg(short = 'F', long = "set-fit-pointer")]
    pub(crate) set_fit_pointer: bool,

    /// FIT type for add operations.
    #[arg(short = 't', long = "fit-type", value_name = "TYPE", value_parser = parse_u8)]
    pub(crate) fit_type: Option<u8>,

    /// CBFS filename or region name for add operations.
    #[arg(
        short = 'n',
        long = "cbfs-filename",
        alias = "name",
        value_name = "NAME"
    )]
    pub(crate) name: Option<String>,

    /// Maximum table size for add operations.
    #[arg(short = 's', long = "max-table-size", value_name = "COUNT")]
    pub(crate) max_table_size: Option<usize>,

    /// Top-swap size for alternate FIT pointer locations.
    #[arg(short = 'j', long = "topswap-size", value_name = "SIZE", value_parser = parse_usize)]
    pub(crate) topswap_size: Option<usize>,

    /// FMAP region containing the CBFS file referenced by add operations.
    #[arg(short = 'R', long = "file-region", value_name = "REGION")]
    pub(crate) file_region: Option<String>,

    /// Increase diagnostic verbosity. Accepted for parity.
    #[arg(short = 'v', long = "verbose", action = ArgAction::Count)]
    pub(crate) verbose: u8,
}

#[derive(Debug, Args)]
pub(crate) struct ElogArgs {
    /// File that holds the event log partition.
    #[arg(short = 'f', long = "file", global = true, value_name = "FILE")]
    pub(crate) file: Option<PathBuf>,

    /// Print timestamps in UTC. Localtime printing currently also uses UTC.
    #[arg(short = 'U', long = "utc", global = true)]
    pub(crate) utc: bool,

    #[command(subcommand)]
    pub(crate) command: ElogCommand,
}

#[derive(Debug, Subcommand)]
pub(crate) enum ElogCommand {
    /// List event log entries.
    List,
    /// Clear all event log entries and append a LOG_CLEAR event.
    Clear,
    /// Add an event to the event log.
    Add {
        /// Event type, decimal or hexadecimal.
        #[arg(value_parser = parse_u8)]
        event_type: u8,
        /// Optional event data as a contiguous hex string.
        event_data: Option<String>,
    },
}

#[derive(Debug, Args)]
pub(crate) struct CseSergerArgs {
    /// CSE image.
    #[arg(value_name = "FILE")]
    pub(crate) image: PathBuf,

    #[command(subcommand)]
    pub(crate) command: CseSergerCommand,
}

#[derive(Debug, Subcommand)]
pub(crate) enum CseSergerCommand {
    /// Print BPDT entries and optionally sub-partition directories.
    Print(CseSergerPrintArgs),
    /// Dump sub-partitions to files.
    Dump(CseSergerDumpArgs),
    /// Print a generated CSE layout for a BPDT version.
    #[command(name = "print-layout")]
    PrintLayout(CseLayoutArgs),
    /// Create a CSE layout file for a BPDT version.
    #[command(name = "create-layout")]
    CreateLayout(CseLayoutArgs),
    /// Create an empty BPDT file for a BPDT version.
    #[command(name = "create-bpdt")]
    CreateBpdt(CseVersionArgs),
}

#[derive(Debug, Args)]
pub(crate) struct CseSergerPrintArgs {
    /// Print a single partition name.
    #[arg(
        short = 'n',
        long = "partition-name",
        alias = "parition_name",
        value_name = "NAME"
    )]
    pub(crate) name: Option<String>,

    /// Print sub-partition directory entries.
    #[arg(short = 's', long = "sub-partition", alias = "sub_partition")]
    pub(crate) sub_partition: bool,

    /// Print a single partition type.
    #[arg(short = 't', long = "type", value_name = "TYPE")]
    pub(crate) partition_type: Option<u32>,
}

#[derive(Debug, Args)]
pub(crate) struct CseSergerDumpArgs {
    /// Directory to dump partition files in.
    #[arg(
        short = 'o',
        long = "output-dir",
        alias = "output_dir",
        value_name = "DIR"
    )]
    pub(crate) output_dir: Option<PathBuf>,

    /// Partition name to dump.
    #[arg(
        short = 'n',
        long = "partition-name",
        alias = "parition_name",
        value_name = "NAME"
    )]
    pub(crate) name: Option<String>,

    /// Partition type to dump.
    #[arg(short = 't', long = "type", value_name = "TYPE")]
    pub(crate) partition_type: Option<u32>,
}

#[derive(Debug, Args)]
pub(crate) struct CseVersionArgs {
    /// BPDT version: 1.6 or 1.7.
    #[arg(short = 'v', long = "version", value_name = "VERSION")]
    pub(crate) version: String,
}

#[derive(Debug, Args)]
pub(crate) struct CseLayoutArgs {
    #[command(flatten)]
    pub(crate) version: CseVersionArgs,

    /// Data partition offset:size.
    #[arg(long = "dp", value_parser = parse_region_arg)]
    pub(crate) data: Option<(usize, usize)>,
    /// BP1 offset:size.
    #[arg(long = "bp1", value_parser = parse_region_arg)]
    pub(crate) bp1: Option<(usize, usize)>,
    /// BP2 offset:size.
    #[arg(long = "bp2", value_parser = parse_region_arg)]
    pub(crate) bp2: Option<(usize, usize)>,
    /// BP3 offset:size.
    #[arg(long = "bp3", value_parser = parse_region_arg)]
    pub(crate) bp3: Option<(usize, usize)>,
    /// BP4 offset:size.
    #[arg(long = "bp4", value_parser = parse_region_arg)]
    pub(crate) bp4: Option<(usize, usize)>,
}

#[derive(Debug, Args)]
pub(crate) struct CseFptArgs {
    /// CSE image containing an FPT.
    #[arg(value_name = "FILE")]
    pub(crate) image: PathBuf,

    #[command(subcommand)]
    pub(crate) command: CseFptCommand,
}

#[derive(Debug, Subcommand)]
pub(crate) enum CseFptCommand {
    /// Print the FPT header and entries, or one named partition.
    Print(CseFptPrintArgs),
    /// Dump valid FPT partitions to files.
    Dump(CseFptDumpArgs),
}

#[derive(Debug, Args)]
pub(crate) struct CseFptPrintArgs {
    /// Name of partition to print.
    #[arg(
        short = 'n',
        long = "partition-name",
        alias = "partition_name",
        value_name = "NAME"
    )]
    pub(crate) partition_name: Option<String>,
}

#[derive(Debug, Args)]
pub(crate) struct CseFptDumpArgs {
    /// Directory to dump partition files in.
    #[arg(
        short = 'o',
        long = "output-dir",
        alias = "output_dir",
        value_name = "OUTPUT_DIR"
    )]
    pub(crate) output_dir: Option<PathBuf>,

    /// Name of partition to dump.
    #[arg(
        short = 'n',
        long = "partition-name",
        alias = "partition_name",
        value_name = "NAME"
    )]
    pub(crate) partition_name: Option<String>,
}

#[derive(Debug, Args)]
pub(crate) struct LayoutArgs {
    /// Include read-only FMAP regions in the listing.
    #[arg(short = 'w', long = "with-readonly")]
    pub(crate) include_readonly: bool,
}

#[derive(Debug, Args)]
pub(crate) struct PrintArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// Use machine-parseable tab-separated output.
    #[arg(short = 'k', long = "mach-parseable")]
    pub(crate) machine_parseable: bool,
}

#[derive(Debug, Args)]
pub(crate) struct ReadArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// Destination file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,
}

#[derive(Debug, Args)]
pub(crate) struct ExtractArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,

    /// Destination file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// Write raw compressed bytes instead of decompressing.
    #[arg(short = 'U', long = "unprocessed")]
    pub(crate) unprocessed: bool,
}

#[derive(Debug, Args)]
pub(crate) struct AddArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    #[command(flatten)]
    pub(crate) placement: EntryPlacementArgs,

    /// Source file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,

    /// CBFS entry type.
    #[arg(short = 't', long = "type", default_value = "raw", value_name = "TYPE")]
    pub(crate) file_type: CbfsType,

    /// Compression algorithm: none, lzma, lz4, or zstd.
    #[arg(
        short = 'c',
        long = "compression",
        default_value = "none",
        value_name = "ALGO"
    )]
    pub(crate) compression: Compression,

    /// Add a vboot hash attribute for the final stored CBFS data.
    #[arg(short = 'A', long = "hash-algorithm", value_name = "ALGO")]
    pub(crate) hash_algorithm: Option<HashAlgorithm>,
}

#[derive(Debug, Args)]
pub(crate) struct AddIntArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    #[command(flatten)]
    pub(crate) placement: EntryPlacementArgs,

    /// Integer value. Decimal and 0x-prefixed hexadecimal are accepted.
    #[arg(short = 'i', long = "int", value_name = "VALUE", value_parser = parse_u64)]
    pub(crate) value: u64,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,
}

#[derive(Debug, Args)]
pub(crate) struct AddFlatBinaryArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    #[command(flatten)]
    pub(crate) placement: EntryPlacementArgs,

    /// Source flat binary file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,

    /// Payload load address. Decimal and 0x-prefixed hexadecimal are accepted.
    #[arg(short = 'l', long = "load-address", value_name = "ADDR", value_parser = parse_u64)]
    pub(crate) load_address: u64,

    /// Payload entry point. Decimal and 0x-prefixed hexadecimal are accepted.
    #[arg(short = 'e', long = "entry-point", value_name = "ADDR", value_parser = parse_u64)]
    pub(crate) entry_point: u64,

    /// Payload segment compression algorithm.
    #[arg(
        short = 'c',
        long = "compression",
        default_value = "none",
        value_name = "ALGO"
    )]
    pub(crate) compression: Compression,

    /// Add a vboot hash attribute for the final stored CBFS data.
    #[arg(short = 'A', long = "hash-algorithm", value_name = "ALGO")]
    pub(crate) hash_algorithm: Option<HashAlgorithm>,
}

#[derive(Debug, Args)]
pub(crate) struct AddPayloadArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    #[command(flatten)]
    pub(crate) placement: EntryPlacementArgs,

    /// Source ELF payload file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,

    /// Payload segment compression algorithm.
    #[arg(
        short = 'c',
        long = "compression",
        default_value = "none",
        value_name = "ALGO"
    )]
    pub(crate) compression: Compression,

    /// Add a vboot hash attribute for the final stored CBFS data.
    #[arg(short = 'A', long = "hash-algorithm", value_name = "ALGO")]
    pub(crate) hash_algorithm: Option<HashAlgorithm>,
}

#[derive(Debug, Args)]
pub(crate) struct AddStageArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    #[command(flatten)]
    pub(crate) placement: EntryPlacementArgs,

    /// Source ELF stage file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,

    /// Section names to ignore, comma separated. Accepted for CLI parity; currently not applied.
    #[arg(short = 'S', long = "ignore-sec", value_name = "SECTIONS")]
    pub(crate) ignore_sections: Option<String>,

    /// Stage compression algorithm: none, lzma, lz4, or zstd.
    #[arg(
        short = 'c',
        long = "compression",
        default_value = "none",
        value_name = "ALGO"
    )]
    pub(crate) compression: Compression,

    /// Add a vboot hash attribute for the final stored CBFS data.
    #[arg(short = 'A', long = "hash-algorithm", value_name = "ALGO")]
    pub(crate) hash_algorithm: Option<HashAlgorithm>,
}

#[derive(Debug, Args)]
pub(crate) struct AddMasterHeaderArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// Top-swap size. Accepted for CLI parity; secondary top-swap header updates are not implemented yet.
    #[arg(short = 'j', long = "topswap-size", value_name = "SIZE", value_parser = parse_size)]
    pub(crate) topswap_size: Option<usize>,
}

#[derive(Debug, Args)]
#[command(group(
    ArgGroup::new("create_source")
        .required(true)
        .args(["size", "flashmap"])
))]
pub(crate) struct CreateArgs {
    /// Image size. Decimal, 0x-prefixed hexadecimal, and K/M/G suffixes are accepted.
    #[arg(short = 's', long = "size", value_name = "SIZE", value_parser = parse_size)]
    pub(crate) size: Option<usize>,

    /// FMAP binary used to create a partitioned firmware image.
    #[arg(short = 'M', long = "flashmap", value_name = "FMAP")]
    pub(crate) flashmap: Option<PathBuf>,

    /// Comma-separated FMAP regions to initialize as CBFS regions with -M.
    #[arg(
        short = 'r',
        long = "fmap-regions",
        value_name = "REGIONS",
        requires = "flashmap"
    )]
    pub(crate) fmap_regions: Option<String>,
}

#[derive(Debug, Args)]
pub(crate) struct WriteArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// Source file.
    #[arg(short = 'f', long = "file", value_name = "FILE")]
    pub(crate) file: PathBuf,

    /// Permit writing into CBFS-looking regions or writing CBFS-looking input.
    #[arg(short = 'F', long = "force")]
    pub(crate) force: bool,

    /// Write smaller input at the beginning of the region.
    #[arg(short = 'u', long = "fill-upward", conflicts_with = "fill_downward")]
    pub(crate) fill_upward: bool,

    /// Write smaller input at the end of the region.
    #[arg(short = 'd', long = "fill-downward")]
    pub(crate) fill_downward: bool,

    /// Fill the whole region with this byte before a partial write.
    #[arg(short = 'i', long = "int", value_name = "BYTE", value_parser = parse_u8)]
    pub(crate) fill: Option<u8>,
}

#[derive(Debug, Args)]
pub(crate) struct RemoveArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// CBFS entry name.
    #[arg(short = 'n', long = "name", value_name = "NAME")]
    pub(crate) name: String,
}

#[derive(Debug, Args)]
pub(crate) struct RegionCommandArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,
}

#[derive(Debug, Args)]
pub(crate) struct CopyArgs {
    #[command(flatten)]
    pub(crate) region: RegionOption,

    /// Source FMAP region name.
    #[arg(short = 'R', long = "source-region", value_name = "REGION")]
    pub(crate) source_region: String,
}

#[derive(Debug, Args)]
pub(crate) struct RegionOption {
    /// FMAP region name, matching cbfstool's -r/--fmap-regions spelling.
    #[arg(short = 'r', long = "fmap-regions", value_name = "REGION")]
    pub(crate) region: Option<String>,
}

#[derive(Debug, Args)]
pub(crate) struct EntryPlacementArgs {
    /// Place the entry at this CBFS-region-relative offset.
    #[arg(short = 'b', long = "base-address", value_name = "OFFSET", value_parser = parse_usize)]
    pub(crate) base_address: Option<usize>,

    /// Place the entry at the first free offset with this alignment.
    #[arg(short = 'a', long = "alignment", value_name = "ALIGN", value_parser = parse_size, conflicts_with = "base_address")]
    pub(crate) alignment: Option<usize>,
}

fn parse_hex_usize(s: &str) -> std::result::Result<usize, String> {
    let hex = s.strip_prefix("0x").unwrap_or(s);
    usize::from_str_radix(hex, 16).map_err(|err| err.to_string())
}

fn parse_usize(s: &str) -> std::result::Result<usize, String> {
    parse_u64(s).and_then(|value| {
        usize::try_from(value).map_err(|_| format!("value {value:#x} does not fit in usize"))
    })
}

fn parse_size(s: &str) -> std::result::Result<usize, String> {
    let (number, multiplier) = match s.as_bytes().last().copied() {
        Some(b'k' | b'K') => (&s[..s.len() - 1], 1024_u64),
        Some(b'm' | b'M') => (&s[..s.len() - 1], 1024_u64 * 1024),
        Some(b'g' | b'G') => (&s[..s.len() - 1], 1024_u64 * 1024 * 1024),
        _ => (s, 1),
    };
    let value = parse_u64(number)?
        .checked_mul(multiplier)
        .ok_or_else(|| format!("size {s} overflows u64"))?;
    usize::try_from(value).map_err(|_| format!("size {s} does not fit in usize"))
}

fn parse_region_arg(s: &str) -> std::result::Result<(usize, usize), String> {
    let (offset, size) = s
        .split_once(':')
        .ok_or_else(|| "expected offset:size".to_owned())?;
    Ok((parse_usize(offset)?, parse_usize(size)?))
}

fn parse_u64(s: &str) -> std::result::Result<u64, String> {
    if let Some(hex) = s.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(|err| err.to_string())
    } else {
        s.parse::<u64>().map_err(|err| err.to_string())
    }
}

fn parse_u8(s: &str) -> std::result::Result<u8, String> {
    parse_u64(s).and_then(|value| {
        u8::try_from(value).map_err(|_| format!("value {value:#x} does not fit in u8"))
    })
}
