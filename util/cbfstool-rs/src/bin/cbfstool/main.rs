// SPDX-License-Identifier: GPL-2.0-only

mod cli;
mod commands;
mod common;
mod fileio;
mod manifest;
mod tools;

use std::env;
use std::ffi::OsString;
use std::fs;
use std::process::ExitCode;

use cbfstool_rs::{FirmwareImage, Result};
use clap::Parser;

use cli::{Cli, Command, ToolCli, ToolCommand};

fn main() -> ExitCode {
    let args = env::args_os().collect::<Vec<_>>();
    let result = if dispatch_tool(&args) {
        run_tool(ToolCli::parse_from(args))
    } else {
        run(Cli::parse_from(args))
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            eprintln!("cbfstool-rs: {err}");
            ExitCode::FAILURE
        }
    }
}

fn dispatch_tool(args: &[OsString]) -> bool {
    args.get(1)
        .and_then(|arg| arg.to_str())
        .is_some_and(ToolCommand::is_tool_name)
}

fn run(cli: Cli) -> Result<()> {
    let _verbosity = cli.verbose;
    if let Command::Create(args) = cli.command {
        return commands::create(&cli.image, args);
    }

    let bytes = fs::read(&cli.image)?;
    let image = FirmwareImage::parse(&bytes)?;

    match cli.command {
        Command::Layout(args) => commands::layout(&image, &args),
        Command::Print(args) => commands::print_cbfs(&image, &args),
        Command::Read(args) => commands::read_region(&image, args),
        Command::Extract(args) => commands::extract(&image, args),
        Command::Add(args) => commands::add(&cli.image, bytes, args),
        Command::AddInt(args) => commands::add_int(&cli.image, bytes, args),
        Command::AddFlatBinary(args) => commands::add_flat_binary(&cli.image, bytes, args),
        Command::AddPayload(args) => commands::add_payload(&cli.image, bytes, args),
        Command::AddStage(args) => commands::add_stage(&cli.image, bytes, args),
        Command::AddMasterHeader(args) => commands::add_master_header(&cli.image, bytes, args),
        Command::Write(args) => commands::write_region(&cli.image, bytes, args),
        Command::Remove(args) => commands::remove(&cli.image, bytes, args),
        Command::Compact(args) => commands::compact(&cli.image, bytes, args),
        Command::Copy(args) => commands::copy(&cli.image, bytes, args),
        Command::Expand(args) => commands::expand(&cli.image, bytes, args),
        Command::Truncate(args) => commands::truncate(&cli.image, bytes, args),
        Command::Create(_) => unreachable!("create handled before image read"),
    }
}

fn run_tool(cli: ToolCli) -> Result<()> {
    match cli.command {
        ToolCommand::BuildImage(args) => manifest::build_image(args),
        ToolCommand::Fmap(args) => tools::fmap_tool(args),
        ToolCommand::CbfsCompression(args) => tools::cbfs_compression_tool(args),
        ToolCommand::AmdCompress(args) => tools::amd_compress_tool(args),
        ToolCommand::Rmod(args) => tools::rmod_tool(args),
        ToolCommand::Ifwi(args) => tools::ifwi_tool(args),
        ToolCommand::Ifit(args) => tools::ifit_tool(args),
        ToolCommand::Elog(args) => tools::elog_tool(args),
        ToolCommand::CseFpt(args) => tools::cse_fpt_tool(args),
        ToolCommand::CseSerger(args) => tools::cse_serger_tool(args),
    }
}
