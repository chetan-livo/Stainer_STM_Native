#!/usr/bin/env python3
"""Generate firmware/.project and firmware/.cproject for STM32CubeIDE.

One managed-build C++ project, one Debug and one Release build configuration
per board. Edit BOARDS (or the shared settings below) and re-run:

    python tools/generate_cubeide_project.py

The generated files are committed. Do not edit them by hand in the IDE and in
this script at the same time; regenerate after changing this file.
"""
from pathlib import Path
import hashlib
from xml.sax.saxutils import quoteattr

PROJECT = 'Stainer_STM_Native'
ROOT = Path(__file__).resolve().parent.parent / 'firmware'

# MCU part -> (CubeIDE target name, device define, startup file, default linker script)
MCUS = {
    'F446ZE': ('STM32F446ZETx', 'STM32F446xx', 'startup_stm32f446xx.s', 'STM32F446ZETX_FLASH.ld'),
    'F407VE': ('STM32F407VETx', 'STM32F407xx', 'startup_stm32f407xx.s', 'STM32F407VETX_FLASH.ld'),
    'F401RC': ('STM32F401RCTx', 'STM32F401xC', 'startup_stm32f401xc.s', 'STM32F401RCTX_FLASH.ld'),
}

# name, MCU, board folder under Boards/, board define (matches the Arduino
# sketch's PCB selection), extra defines, linker script override
BOARDS = [
    ('Master',      'F446ZE', 'Master',      'Stainer_Master_PCB',      [], None),
    ('Nozzle',      'F407VE', 'Nozzle',      'Nozzle_Mount_PCB',        [], None),
    ('Gantry',      'F407VE', 'Gantry',      'Stainer_Gantry_PCB_UART', [], None),
    ('Magazine1',   'F446ZE', 'Magazine',    'Magazine1_IR_PCB_UART',
        ['USER_VECT_TAB_ADDRESS', 'VECT_TAB_OFFSET=0x20000', 'MAGAZINE_APP_OFFSET_BUILD'],
        'STM32F446ZETX_APP_0x20000.ld'),
    ('Magazine2',   'F446ZE', 'Magazine',    'Magazine2_IR_PCB_UART',
        ['USER_VECT_TAB_ADDRESS', 'VECT_TAB_OFFSET=0x20000', 'MAGAZINE_APP_OFFSET_BUILD'],
        'STM32F446ZETX_APP_0x20000.ld'),
    ('GantryXHall', 'F446ZE', 'GantryXHall', 'Gantry_X_Hall_PCB',       [], None),
    ('GantryZHall', 'F401RC', 'GantryZHall', 'Gantry_Z_Hall_PCB',       [], None),
]

VARIANTS = {
    # name: (debug build?, optimisation, debug level)
    'Debug':   (True,  'og', 'g3'),
    'Release': (False, 'o2', 'g'),   # symbols kept for post-mortem debugging
}

INCLUDES = [
    '../App', '../Boards', '../Platform/Inc',
    '../Drivers/CMSIS/Include',
    '../Drivers/CMSIS/Device/ST/STM32F4xx/Include',
    '../Drivers/STM32F4xx_HAL_Driver/Inc',
    '../Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
    '../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc',
]
COMMON_DEFINES = ['USE_HAL_DRIVER', 'USE_FULL_LL_DRIVER']

ST = 'com.st.stm32cube.ide.mcu.gnu.managedbuild'


def uid(*parts):
    return str(int(hashlib.sha1('|'.join(parts).encode()).hexdigest()[:8], 16) % 2000000000)


def opt(key, oid, value, vtype='enumerated'):
    return f'<option id="{oid}.{uid(key, oid)}" superClass="{oid}" value={quoteattr(value)} valueType="{vtype}"/>'


def list_opt(key, oid, vtype, values):
    rows = ''.join(f'\n\t\t\t\t\t\t\t\t\t<listOptionValue builtIn="false" value={quoteattr(v)}/>' for v in values)
    return (f'<option IS_BUILTIN_EMPTY="false" IS_VALUE_EMPTY="false" id="{oid}.{uid(key, oid)}" '
            f'superClass="{oid}" valueType="{vtype}">{rows}\n\t\t\t\t\t\t\t\t</option>')


def configuration(board, variant):
    name, mcu, folder, board_define, extra, ld_override = board
    debug, optim, dbg = VARIANTS[variant]
    target, device, startup, ld = MCUS[mcu]
    ld = ld_override or ld
    cfg = f'{name}-{variant}'
    kind = 'debug' if debug else 'release'
    cid = f'{ST}.config.exe.{kind}.{uid(cfg)}'
    defines = (['DEBUG'] if debug else []) + COMMON_DEFINES + [device, board_define] + extra
    script = '${workspace_loc:/${ProjName}/Platform/LinkerScripts/' + ld + '}'
    other_startups = [f'Platform/Startup/{s}' for _, _, s, _ in MCUS.values() if s != startup]
    other_boards = sorted({f'Boards/{b[2]}' for b in BOARDS if b[2] != folder})
    excluding = '|'.join(other_startups + other_boards)
    defaults = (f'com.st.stm32cube.ide.common.services.build.inputs.revA.1.0.6 || {cfg} || {str(debug).lower()} || '
                f'Executable || {ST}.option.toolchain.value.workspace || {target} || 0 || 0 || arm-none-eabi- || '
                '${gnu_tools_for_stm32_compiler_path} || ' + ' | '.join(INCLUDES) + ' ||  ||  || '
                + ' | '.join(defines) + ' ||  ||  ||  ||  || ' + script + ' || true || NonSecure ||  ||  ||  || None || ')
    t = lambda tool: f'{ST}.tool.{tool}'
    co = lambda tool, o: f'{ST}.tool.{tool}.option.{o}'
    return f'''		<cconfiguration id="{cid}">
			<storageModule buildSystemId="org.eclipse.cdt.managedbuilder.core.configurationDataProvider" id="{cid}" moduleId="org.eclipse.cdt.core.settings" name="{cfg}">
				<externalSettings/>
				<extensions>
					<extension id="org.eclipse.cdt.core.ELF" point="org.eclipse.cdt.core.BinaryParser"/>
					<extension id="org.eclipse.cdt.core.GASErrorParser" point="org.eclipse.cdt.core.ErrorParser"/>
					<extension id="org.eclipse.cdt.core.GmakeErrorParser" point="org.eclipse.cdt.core.ErrorParser"/>
					<extension id="org.eclipse.cdt.core.GLDErrorParser" point="org.eclipse.cdt.core.ErrorParser"/>
					<extension id="org.eclipse.cdt.core.CWDLocator" point="org.eclipse.cdt.core.ErrorParser"/>
					<extension id="org.eclipse.cdt.core.GCCErrorParser" point="org.eclipse.cdt.core.ErrorParser"/>
				</extensions>
			</storageModule>
			<storageModule moduleId="cdtBuildSystem" version="4.0.0">
				<configuration artifactExtension="elf" artifactName="${{ProjName}}-{name}" buildArtefactType="org.eclipse.cdt.build.core.buildArtefactType.exe" buildProperties="org.eclipse.cdt.build.core.buildArtefactType=org.eclipse.cdt.build.core.buildArtefactType.exe,org.eclipse.cdt.build.core.buildType=org.eclipse.cdt.build.core.buildType.{kind}" cleanCommand="rm -rf" description="{name} board, {variant}" id="{cid}" name="{cfg}" parent="{ST}.config.exe.{kind}">
					<folderInfo id="{cid}." name="/" resourcePath="">
						<toolChain id="{ST}.toolchain.exe.{kind}.{uid(cfg, 'tc')}" name="MCU ARM GCC" superClass="{ST}.toolchain.exe.{kind}">
							{opt(cfg, ST + '.option.target_mcu', target, 'string')}
							{opt(cfg, ST + '.option.target_cpuid', '0', 'string')}
							{opt(cfg, ST + '.option.target_coreid', '0', 'string')}
							{opt(cfg, ST + '.option.fpu', ST + '.option.fpu.value.fpv4-sp-d16')}
							{opt(cfg, ST + '.option.floatabi', ST + '.option.floatabi.value.hard')}
							{opt(cfg, ST + '.option.target_board', 'genericBoard', 'string')}
							{opt(cfg, ST + '.option.defaults', defaults, 'string')}
							{opt(cfg, ST + '.option.converthex', 'true', 'boolean')}
							{opt(cfg, ST + '.option.convertbinary', 'true', 'boolean')}
							<targetPlatform archList="all" binaryParser="org.eclipse.cdt.core.ELF" id="{ST}.targetplatform.{uid(cfg, 'tp')}" isAbstract="false" osList="all" superClass="{ST}.targetplatform"/>
							<builder buildPath="${{workspace_loc:/{PROJECT}}}/{cfg}" id="{ST}.builder.{uid(cfg, 'b')}" keepEnvironmentInBuildfile="false" managedBuildOn="true" name="Gnu Make Builder.{cfg}" parallelBuildOn="true" parallelizationNumber="optimal" superClass="{ST}.builder"/>
							<tool id="{t('assembler')}.{uid(cfg, 'as')}" name="MCU GCC Assembler" superClass="{t('assembler')}">
								{opt(cfg, co('assembler', 'debuglevel'), co('assembler', 'debuglevel.value.' + dbg))}
								{list_opt(cfg + 'as', co('assembler', 'definedsymbols'), 'definedSymbols', defines)}
								<inputType id="{t('assembler')}.input.{uid(cfg, 'asi')}" superClass="{t('assembler')}.input"/>
							</tool>
							<tool id="{t('c.compiler')}.{uid(cfg, 'cc')}" name="MCU GCC Compiler" superClass="{t('c.compiler')}">
								{opt(cfg, co('c.compiler', 'debuglevel'), co('c.compiler', 'debuglevel.value.' + dbg))}
								{opt(cfg, co('c.compiler', 'optimization.level'), co('c.compiler', 'optimization.level.value.' + optim))}
								{opt(cfg, co('c.compiler', 'languagestandard'), co('c.compiler', 'languagestandard.value.gnu11'))}
								{list_opt(cfg + 'cc', co('c.compiler', 'definedsymbols'), 'definedSymbols', defines)}
								{list_opt(cfg + 'cc', co('c.compiler', 'includepaths'), 'includePath', INCLUDES)}
								<inputType id="{t('c.compiler')}.input.c.{uid(cfg, 'cci')}" superClass="{t('c.compiler')}.input.c"/>
							</tool>
							<tool id="{t('cpp.compiler')}.{uid(cfg, 'cpp')}" name="MCU G++ Compiler" superClass="{t('cpp.compiler')}">
								{opt(cfg, co('cpp.compiler', 'debuglevel'), co('cpp.compiler', 'debuglevel.value.' + dbg))}
								{opt(cfg, co('cpp.compiler', 'optimization.level'), co('cpp.compiler', 'optimization.level.value.' + optim))}
								{opt(cfg, co('cpp.compiler', 'languagestandard'), co('cpp.compiler', 'languagestandard.value.gnupp17'))}
								{opt(cfg, co('cpp.compiler', 'noexceptions'), 'true', 'boolean')}
								{opt(cfg, co('cpp.compiler', 'nortti'), 'true', 'boolean')}
								{opt(cfg, co('cpp.compiler', 'nothreadsafestatics'), 'true', 'boolean')}
								{opt(cfg, co('cpp.compiler', 'nousecxaatexit'), 'true', 'boolean')}
								{opt(cfg, co('cpp.compiler', 'warnings.extra'), 'true', 'boolean')}
								{list_opt(cfg + 'cpp', co('cpp.compiler', 'definedsymbols'), 'definedSymbols', defines)}
								{list_opt(cfg + 'cpp', co('cpp.compiler', 'includepaths'), 'includePath', INCLUDES)}
								<inputType id="{t('cpp.compiler')}.input.cpp.{uid(cfg, 'cppi')}" superClass="{t('cpp.compiler')}.input.cpp"/>
							</tool>
							<tool id="{t('c.linker')}.{uid(cfg, 'cl')}" name="MCU GCC Linker" superClass="{t('c.linker')}">
								{opt(cfg, co('c.linker', 'script'), script, 'string')}
							</tool>
							<tool id="{t('cpp.linker')}.{uid(cfg, 'cppl')}" name="MCU G++ Linker" superClass="{t('cpp.linker')}">
								{opt(cfg, co('cpp.linker', 'script'), script, 'string')}
								<inputType id="{t('cpp.linker')}.input.{uid(cfg, 'cppli')}" superClass="{t('cpp.linker')}.input">
									<additionalInput kind="additionalinputdependency" paths="$(USER_OBJS)"/>
									<additionalInput kind="additionalinput" paths="$(LIBS)"/>
								</inputType>
							</tool>
							<tool id="{t('archiver')}.{uid(cfg, 'ar')}" name="MCU GCC Archiver" superClass="{t('archiver')}"/>
							<tool id="{t('size')}.{uid(cfg, 'sz')}" name="MCU Size" superClass="{t('size')}"/>
							<tool id="{t('objdump.listfile')}.{uid(cfg, 'ls')}" name="MCU Output Converter list file" superClass="{t('objdump.listfile')}"/>
							<tool id="{t('objcopy.hex')}.{uid(cfg, 'hx')}" name="MCU Output Converter Hex" superClass="{t('objcopy.hex')}"/>
							<tool id="{t('objcopy.binary')}.{uid(cfg, 'bn')}" name="MCU Output Converter Binary" superClass="{t('objcopy.binary')}"/>
							<tool id="{t('objcopy.verilog')}.{uid(cfg, 'vl')}" name="MCU Output Converter Verilog" superClass="{t('objcopy.verilog')}"/>
							<tool id="{t('objcopy.srec')}.{uid(cfg, 'sr')}" name="MCU Output Converter Motorola S-rec" superClass="{t('objcopy.srec')}"/>
							<tool id="{t('objcopy.symbolsrec')}.{uid(cfg, 'ss')}" name="MCU Output Converter Motorola S-rec with symbols" superClass="{t('objcopy.symbolsrec')}"/>
						</toolChain>
					</folderInfo>
					<sourceEntries>
						<entry excluding={quoteattr(excluding)} flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name=""/>
					</sourceEntries>
				</configuration>
			</storageModule>
			<storageModule moduleId="org.eclipse.cdt.core.externalSettings"/>
		</cconfiguration>
'''


def cproject():
    configs = ''.join(configuration(b, v) for b in BOARDS for v in VARIANTS)
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<?fileVersion 4.0.0?><cproject storage_type_id="org.eclipse.cdt.core.XmlProjectDescriptionStorage">
	<!-- Generated by tools/generate_cubeide_project.py. Edit the script, then regenerate. -->
	<storageModule moduleId="org.eclipse.cdt.core.settings">
{configs}	</storageModule>
	<storageModule moduleId="org.eclipse.cdt.core.pathentry"/>
	<storageModule moduleId="cdtBuildSystem" version="4.0.0">
		<project id="{PROJECT}.null.{uid(PROJECT)}" name="{PROJECT}"/>
	</storageModule>
	<storageModule moduleId="org.eclipse.cdt.core.LanguageSettingsProviders"/>
	<storageModule moduleId="scannerConfiguration">
		<autodiscovery enabled="true" problemReportingEnabled="true" selectedProfileId=""/>
	</storageModule>
</cproject>
'''


PROJECT_XML = f'''<?xml version="1.0" encoding="UTF-8"?>
<projectDescription>
	<name>{PROJECT}</name>
	<comment>Generated by tools/generate_cubeide_project.py</comment>
	<projects>
	</projects>
	<buildSpec>
		<buildCommand>
			<name>org.eclipse.cdt.managedbuilder.core.genmakebuilder</name>
			<triggers>clean,full,incremental,</triggers>
			<arguments>
			</arguments>
		</buildCommand>
		<buildCommand>
			<name>org.eclipse.cdt.managedbuilder.core.ScannerConfigBuilder</name>
			<triggers>full,incremental,</triggers>
			<arguments>
			</arguments>
		</buildCommand>
	</buildSpec>
	<natures>
		<nature>com.st.stm32cube.ide.mcu.MCUProjectNature</nature>
		<nature>org.eclipse.cdt.core.cnature</nature>
		<nature>org.eclipse.cdt.core.ccnature</nature>
		<nature>com.st.stm32cube.ide.mcu.MCUCubeIdeServicesRevAev2ProjectNature</nature>
		<nature>com.st.stm32cube.ide.mcu.MCUSingleCpuProjectNature</nature>
		<nature>org.eclipse.cdt.managedbuilder.core.managedBuildNature</nature>
		<nature>org.eclipse.cdt.managedbuilder.core.ScannerConfigNature</nature>
	</natures>
</projectDescription>
'''

if __name__ == '__main__':
    (ROOT / '.cproject').write_text(cproject(), encoding='utf-8', newline='\n')
    (ROOT / '.project').write_text(PROJECT_XML, encoding='utf-8', newline='\n')
    names = [f'{b[0]}-{v}' for b in BOARDS for v in VARIANTS]
    print(f'Wrote {len(names)} configurations: ' + ', '.join(names))
