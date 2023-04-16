#!/usr/bin/perl

use warnings;
use strict;
use autodie;
use Getopt::Long;
use 5.010;

my $line_num = 0;
my $line;
my $input_file;
my $output_file;
my $array_name = "bqfs_image";

# Remove both leading and trailing whitespace
sub trim
{
    my @array = @_ ;
    for(@array){
        s/^\s+|\s+$//g ;
    }
    return @array ;
}


sub output_title
{
	select OUTPUTFILE;
	say  "/*****************************************************************************";
 	say  "* Copyright 2010 Texas Instruments Corporation, All Rights Reserved.";
 	say  "* TI makes NO WARRANTY as to software products, which are supplied \"AS-IS\"";
	say  "*****************************************************************************/";
	say  "";
	say  "/*****************************************************************************";
	say  "*     This code is automatically generated from bqfs/dffs file.              *";
 	say  "*          DO NOT MODIFY THIS FILE DIRECTLY                                  *";
	say  "*****************************************************************************/";
	say  "";
	say	 "//this file is generated from $input_file at ".localtime;
	say  "";
	select STDOUT;
}

sub output_header
{
	select OUTPUTFILE;
	say  "#ifndef __BQFS_FILE__";
	say  "#define __BQFS_FILE__";
	say  "";
	say  "#include \"bqfs_cmd_type.h\"";
	say  "";
	select STDOUT;
}

sub output_bqfs_img_head
{
	say OUTPUTFILE "const bqfs_cmd_t ".$array_name."[] = {";
}

sub output_bqfs_img_tail
{
	select OUTPUTFILE;
	say  "};";
	say  "//end of const bqfs_cmd_t ".$array_name."[]";
	say  "#endif";
	select STDOUT;
}

sub output_bqfs_cmd_read
{
	select OUTPUTFILE;
	printf  "\t{\n";
	printf  "\t\t.cmd_type\t= CMD_R,\n";
	printf  "\t\t.addr\t\t= 0x".shift(@_).",\n";
	printf  "\t\t.reg\t\t= 0x".shift(@_).",\n";
	if($line){
		printf  "\t\t.line_num\t= $line_num,\n";
	}
	printf  "\t},\n";
	select STDOUT;

}

sub output_bqfs_cmd_write
{
	my $i = 0;
	
	select OUTPUTFILE;
	
	printf  "\t{\n";
	printf  "\t\t.cmd_type\t= CMD_W,\n";
	printf  "\t\t.addr\t\t= 0x".shift(@_).",\n";
	printf  "\t\t.reg\t\t= 0x".shift(@_).",\n";
	printf  "\t\t.data\t\t= {.bytes = {";

	foreach my $data (@_){
		if($i != scalar(@_) - 1){
			printf  "0x$data, ";
		}
		else{#last one
			printf  "0x$data";
		}
		$i = $i + 1;
	}

	printf  "}},\n";
	printf  "\t\t.data_len\t= $i,\n";
	if($line){
		printf  "\t\t.line_num\t= $line_num,\n";
	}
	printf  "\t},\n";
	
	select STDOUT;

}

sub output_bqfs_cmd_compare
{
	my $i = 0;

	select OUTPUTFILE;
	
	printf  "\t{\n";
	printf  "\t\t.cmd_type\t= CMD_C,\n";
	printf  "\t\t.addr\t\t= 0x".shift(@_).",\n";
	printf  "\t\t.reg\t\t= 0x".shift(@_).",\n";
	printf  "\t\t.data\t\t= {.bytes = {";
	foreach my $data (@_){
		if($i != scalar(@_) - 1){
			printf  "0x$data, ";
		}
		else{#last one
			printf  "0x$data";
		}
		$i++;
	}
	printf  "}},\n";
	printf  "\t\t.data_len\t= $i,\n";
	if($line){
		printf  "\t\t.line_num\t= $line_num,\n";
	}
	printf  "\t},\n";
	
	select STDOUT;
}



sub output_bqfs_cmd_wait
{
	select OUTPUTFILE;
	
	printf  "\t{\n";
	printf  "\t\t.cmd_type\t= CMD_X,\n";
	printf  "\t\t.data\t\t= {.delay = ".shift(@_)."},\n";
	if($line){
		printf  "\t\t.line_num\t= $line_num,\n";
	}
	printf  "\t},\n";
	
	select STDOUT;

}

sub parse_argument
{
	GetOptions(
		'line!'		=> \$line,
		'input:s'	=> \$input_file,
		'output:s'	=> \$output_file,
		'name:s'	=> \$array_name,
	);
	if(!$output_file){
		$output_file = $input_file."\.h";
	}

	if($input_file){
		return 1;
	}
	return 0;
		
}


#main function
if (!parse_argument()){
	say "Usage:\n\t $0 --input <input file> [--output {output file}] [--line] [--name {array name}]";
	say "\t\t if output file is not specified, default name is same with input file name with .h extension";
	say "Example:\n\t $0 --input bq27320.df.fs";
	say "\t $0 --input bq27320.df.fs --output bq27320-bqfs-image.h";
	say "\t $0 --input bq27320.df.fs --output bq27320-bqfs-image.h --line";
	say "\t $0 --input bq27320.df.fs --output bq27320-bqfs-image.h --name bqfs_image";
	exit;
}
	
open INPUTFILE, '<', $input_file;
open OUTPUTFILE, '>', $output_file;

&output_title();
&output_header();

&output_bqfs_img_head();
$| = 1;
print "Begin converting";

while(<INPUTFILE>){
	$line_num++; 
	print ".";
	chomp();
	my @line = split(/ /);
	@line = trim(@line);
	my $token = shift(@line);

	if($token eq "R:"){
		&output_bqfs_cmd_read(@line);
		next;
	}
	if($token eq "W:"){
		&output_bqfs_cmd_write(@line);
		next;
	}
	if($token eq "X:"){
		&output_bqfs_cmd_wait(@line);
		next;
	}	
	if ($token eq "C:"){
		&output_bqfs_cmd_compare(@line);
		next;
	}
}

print "Done\n";

&output_bqfs_img_tail();

close INPUTFILE;
close OUTPUTFILE;
