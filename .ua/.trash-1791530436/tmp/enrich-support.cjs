const fs = require('fs');
const path = require('path');
const root = process.cwd();
const ua = path.join(root, '.ua');
const batches = JSON.parse(fs.readFileSync(path.join(ua, 'intermediate/batches.json'), 'utf8')).batches;
const inventory = JSON.parse(fs.readFileSync(path.join(ua, 'intermediate/scan-result.json'), 'utf8'));
const typeFor = f => f.fileCategory === 'docs' ? 'document' : f.fileCategory === 'config' ? 'config' : 'file';
const fileId = p => { const f = inventory.files.find(x => x.path === p); if (!f) throw Error(p); return `${typeFor(f)}:${p}`; };
const summaries = {
 'README.md': ['Explains the PCIe-8332 UDP backend, independent client, installation and hardware-free regression suite. Distinguishes the retained direct-NIC tools and incompatible legacy UE protocol.', ['documentation','entry-point','architecture']],
 'UE接入.md': ['Documents the optional legacy three-axis UDP bridge, explicit run_three_axis startup, session handshake and Unreal integration. Targets are motor angles rather than platform pitch, roll or heave; this protocol is incompatible with the current v1 backend.', ['documentation','legacy-protocol','unreal-engine']],
 'backend.config.example.json': ['Example server endpoint, heartbeat, authentication and ADLINK APS configuration. Maps each axis extension limit to DI2 and retraction limit to DI1 and leaves unit scaling configurable.', ['configuration','aps','limit-inputs']],
 'requirements-legacy.txt': ['Adds pysoem to the backend dependency set for retained direct-NIC tools and their regression tests.', ['dependencies','legacy-ethercat','installation']],
 'requirements.txt': ['Declares pystray and Pillow for the Windows tray backend; the standalone UDP client does not need these packages.', ['dependencies','tray','installation']],
 '使用说明.md': ['Retained direct-NIC debugging guide covering supported A3 motor parameters, wiring, manual continuous targets and cleanup. Explains that the current root launcher opens the new backend and directs users to the UDP client guide.', ['documentation','legacy-ethercat','motion-control']],
 '后台说明.md': ['Deployment and operating guide for the UDP backend, PCIe-8332 configuration, position units and dual-ended limit feedback. Includes SDK usage, logs and regression verification.', ['documentation','aps','deployment']],
 '启动后台.cmd': ['Starts backend.py with the project virtual environment windowed Python after verifying that the interpreter exists.', ['entry-point','windows','tray']],
 '启动后台控制台.cmd': ['Starts backend.py in headless console mode with the project virtual environment and keeps the console visible after exit.', ['entry-point','windows','headless']],
 '启动源码.cmd': ['Checks the project virtual environment and tray imports before launching backend.py with pythonw. Failure labels explain environment and dependency setup.', ['entry-point','windows','installation']],
 '安装后台依赖.cmd': ['Installs backend requirements into the project virtual environment and displays the command result in a console.', ['installation','windows','dependencies']],
 'UdpControl/PROTOCOL.md': ['Defines the v1 JSON UDP RPC contract, exclusive control leases, control and motion sequences, ACK deduplication and telemetry ordering. Covers heartbeat expiry, run isolation and dual-ended limit behavior.', ['documentation','udp-protocol','session-control']],
 'UdpControl/README.md': ['Describes deployment and operation of the hardware-independent Tkinter frontend and standard-library MotorClient SDK. Documents endpoint configuration, selected-axis targets, limit indicators and separate client/server logs.', ['documentation','udp-client','deployment']],
 'UdpControl/config.example.json': ['Example client endpoint and authentication key; hardware settings belong to the backend configuration.', ['configuration','udp-client','authentication']],
 'UdpControl/requirements.txt': ['Documents that the independent GUI and UDP SDK require only standard Python including Tkinter, without hardware or tray packages.', ['dependencies','udp-client','deployment']],
 'UdpControl/启动界面.cmd': ['Launches the independent UdpControl package using the parent virtual environment when available, otherwise the Windows Python launcher.', ['entry-point','windows','udp-client']],
 'UE/SV635NMotionComponent.cpp': ['Implements a tick-driven Unreal UDP client for the legacy three-axis bridge, including handshakes, ordered targets, feedback and timeout disable. It is a reference integration and does not implement the current v1 RPC protocol.', ['unreal-engine','legacy-protocol','udp-client']],
 'UE/SV635NMotionComponent.h': ['Declares the Blueprint-spawnable motor component, endpoint settings, enable/feedback state and three motor-angle targets. The component derives from Unreal UActorComponent and retains its own socket/session state.', ['unreal-engine','component','type-definition']],
 'UdpControl/__init__.py': ['Defines PROTOCOL_VERSION=1 for the independent UDP frontend and SDK package without importing hardware code.', ['entry-point','udp-protocol','constant']],
 'test_udp_layout.py': ['Verifies standalone client deployment without backend packages, backend imports without frontend packages and matching client/server JSON codecs.', ['test','deployment','component-isolation']],
 'ue_demo.py': ['Command-line demonstration of the legacy bridge: performs a hello handshake, streams three sinusoidal motor-angle targets and disables on exit. Requires an explicitly started run_three_axis bridge rather than the current v1 backend.', ['example','legacy-protocol','udp-client']]
};
const methodSummaries = {
 USV635NMotionComponent: 'Enables Unreal component ticking for periodic legacy bridge communication.',
 BeginPlay: 'Validates the endpoint and creates a nonblocking UDP socket bound to an ephemeral client port.',
 SetMotorTargets: 'Stores three finite motor-angle targets and revokes enable if a target is non-finite.',
 SetMotorEnable: 'Sets requested enable state and immediately sends a disable frame when a session exists.',
 SendJson: 'Serializes a JSON object to UTF-8 and sends it through the component socket, reporting send failure.',
 SendHello: 'Sends the unversioned hello packet used to discover a legacy bridge session.',
 SendCommand: 'Sends a legacy command with session, increasing sequence, enable state and three target angles.',
 ReceiveFeedback: 'Processes at most sixteen feedback datagrams per frame, validates the sender and updates session, mapped orders, enable and angle feedback.',
 TickComponent: 'Receives feedback, revokes enable after feedback timeout, refreshes handshakes and sends targets at 30 Hz.',
 EndPlay: 'Requests disable, closes the UDP socket and releases Unreal socket resources.',
 main: 'Validates demo timing and amplitude, discovers a legacy bridge session, streams three sinusoidal targets and disables in a finally block.'
};
for (const idx of [4,5,6]) {
 const b = batches.find(x => x.batchIndex === idx);
 const extraction = JSON.parse(fs.readFileSync(path.join(ua, `tmp/ua-file-extract-results-${idx}.json`), 'utf8'));
 if (extraction.filesUnreadable.length) throw Error('Unreadable support files');
 const nodes = [], edges = [];
 const edge = (s,t,type,w) => { if (s !== t) edges.push({source:s,target:t,type,direction:'forward',weight:w}); };
 for (const f of b.files) {
  const [summary,tags] = summaries[f.path];
  nodes.push({id:fileId(f.path),type:typeFor(f),name:path.posix.basename(f.path),filePath:f.path,summary,tags,complexity:f.sizeLines>200?'complex':f.sizeLines>50?'moderate':'simple'});
  for (const p of b.batchImportData[f.path] || []) edge(fileId(f.path),fileId(p),'imports',.7);
  const e = extraction.results.find(x => x.path === f.path);
  for (const c of e?.classes || []) {
   if (c.methods.length<2 && c.endLine-c.startLine+1<20) continue;
   const id = `class:${f.path}:${c.name}`;
   nodes.push({id,type:'class',name:c.name,filePath:f.path,lineRange:[c.startLine,c.endLine],summary:c.name==='LayoutTests'?'Regression suite proving frontend/backend deployment isolation and wire-format compatibility.':'Blueprint-facing Unreal component owning the legacy UDP bridge socket, handshake and motor feedback state.',tags:c.name==='LayoutTests'?['test','deployment','component-isolation']:['unreal-engine','component','legacy-protocol'],complexity:'moderate'});
   edge(fileId(f.path),id,'contains',1); edge(fileId(f.path),id,'exports',.8);
  }
  const lookup = new Map();
  for (const fn of e?.functions || []) {
   const exported=(e.exports||[]).some(x=>x.name===fn.name);
   if (!exported && fn.endLine-fn.startLine+1<10) continue;
   const name=f.path.endsWith('.cpp')?`USV635NMotionComponent.${fn.name}`:fn.name;
   const id=`function:${f.path}:${name}`; lookup.set(fn.name,id);
   nodes.push({id,type:'function',name,filePath:f.path,lineRange:[fn.startLine,fn.endLine],summary:methodSummaries[fn.name],tags:f.path.endsWith('.cpp')?['unreal-engine','legacy-protocol','udp-client']:['example','legacy-protocol','motion-control'],complexity:fn.endLine-fn.startLine>40?'moderate':'simple'});
   edge(fileId(f.path),id,'contains',1);
   if (exported) edge(fileId(f.path),id,'exports',.8);
   if (f.path.endsWith('.cpp')) edge('class:UE/SV635NMotionComponent.h:USV635NMotionComponent',id,'contains',1);
  }
  for (const call of e?.callGraph || []) if (lookup.has(call.caller)&&lookup.has(call.callee)) edge(lookup.get(call.caller),lookup.get(call.callee),'calls',.8);
 }
 const link=(s,targets,t,w)=>targets.forEach(p=>edge(fileId(s),fileId(p),t,w));
 if(idx===4){
  link('README.md',['backend.py','motor_service.py','aps_backend.py','udp_server.py','UdpControl/__main__.py','app.py','platform_control.py'],'documents',.5);
  link('后台说明.md',['backend.py','motor_service.py','aps_backend.py','udp_server.py','backend.config.example.json','UdpControl/client.py'],'documents',.5);
  link('使用说明.md',['app.py','cli.py','core.py','continuous_control.py'],'documents',.5);
  link('UE接入.md',['platform_control.py','ue_demo.py','UE/SV635NMotionComponent.h','UE/SV635NMotionComponent.cpp'],'documents',.5);
  link('backend.config.example.json',['backend.py','aps_backend.py','udp_server.py','motor_service.py'],'configures',.6);
  link('requirements.txt',['backend.py'],'configures',.6);
  link('requirements-legacy.txt',['requirements.txt'],'depends_on',.6);
  link('requirements-legacy.txt',['core.py','app.py','cli.py'],'configures',.6);
  for(const p of ['启动后台.cmd','启动后台控制台.cmd','启动源码.cmd']) link(p,['backend.py'],'triggers',.6);
  link('安装后台依赖.cmd',['requirements.txt'],'depends_on',.6);
  link('启动源码.cmd',['requirements.txt'],'depends_on',.6);
  for(const [name,start,end,summary] of [['missing_env',10,14,'Explains how to create the project virtual environment and install backend dependencies.'],['missing_dependencies',16,18,'Reports missing backend imports and prints the dependency installation command.'],['failed',20,22,'Pauses after launcher validation failure and returns a nonzero exit status.']]) {
   const id=`function:启动源码.cmd:${name}`;
   nodes.push({id,type:'function',name,filePath:'启动源码.cmd',lineRange:[start,end],summary,tags:['windows','installation','error-handling'],complexity:'simple'});
   edge(fileId('启动源码.cmd'),id,'contains',1);
  }
 }
 if(idx===5){
  link('UdpControl/PROTOCOL.md',['udp_server.py','motor_service.py','UdpControl/client.py','UdpControl/wire.py'],'documents',.5);
  link('UdpControl/README.md',['UdpControl/__main__.py','UdpControl/debug_ui.py','UdpControl/client.py','UdpControl/example_client.py','UdpControl/config.example.json','UdpControl/requirements.txt'],'documents',.5);
  link('UdpControl/config.example.json',['UdpControl/__main__.py','UdpControl/client.py'],'configures',.6);
  link('UdpControl/启动界面.cmd',['UdpControl/__main__.py'],'triggers',.6);
 }
 if(idx===6){
  link('ue_demo.py',['platform_control.py'],'depends_on',.6);
  link('UE/SV635NMotionComponent.cpp',['platform_control.py'],'depends_on',.6);
  for(const p of ['backend.py','motor_service.py','udp_server.py','UdpControl/__main__.py','UdpControl/client.py','UdpControl/debug_ui.py','UdpControl/wire.py']) edge(fileId(p),fileId('test_udp_layout.py'),'tested_by',.5);
  const tests=[['test_client_folder_runs_without_backend_or_site_packages',11,32,'Copies the client package into an isolated directory and proves it imports and exposes CLI help without backend or site packages.'],['test_backend_does_not_import_frontend',34,44,'Uses a subprocess to prove backend imports do not load Tkinter, UdpControl or pysoem.'],['test_wire_format_matches_across_process_components',46,51,'Checks both encoder/decoder directions with hello and target packets to prove shared v1 wire compatibility.']];
  for(const [name,start,end,summary] of tests){const id=`function:test_udp_layout.py:LayoutTests.${name}`;nodes.push({id,type:'function',name:`LayoutTests.${name}`,filePath:'test_udp_layout.py',lineRange:[start,end],summary,tags:['test','deployment','component-isolation'],complexity:'simple'});edge(fileId('test_udp_layout.py'),id,'contains',1);edge('class:test_udp_layout.py:LayoutTests',id,'contains',1);}
 }
 const dedup=[...new Map(edges.map(e=>[`${e.source}|${e.target}|${e.type}`,e])).values()];
 const output=path.join(ua,`intermediate/batch-${idx}.json`);
 fs.writeFileSync(output,JSON.stringify({nodes,edges:dedup},null,2));
 console.log(`Batch ${idx}: ${nodes.length} nodes, ${dedup.length} edges; ${extraction.filesSkipped.length} unsupported files read and covered manually.`);
}
