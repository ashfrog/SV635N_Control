import fs from 'node:fs';
const root='M:/GitHub/SV635N/SV635N_Control';
const batch=JSON.parse(fs.readFileSync(`${root}/.ua/intermediate/batches.json`,'utf8').replace(/^\uFEFF/,'')).batches.find(x=>x.batchIndex===2);
const extraction=JSON.parse(fs.readFileSync(`${root}/.ua/tmp/ua-file-extract-results-2.json`,'utf8'));
if(!extraction.scriptCompleted || extraction.filesUnreadable.length) throw Error('Incomplete extraction');
const fileSemantics={
'UdpControl/__main__.py':['Launches the independent Tk UDP control client with endpoint configuration, command-line overrides, rotating logs, and UI-thread cleanup. It imports no controller or hardware library.',['entry-point','configuration','logging','udp-client']],
'UdpControl/client.py':['Implements the standard-library-only, thread-safe UDP motor SDK with retried ACK requests, owner sessions, per-run targets, independent heartbeats, and telemetry ordering. Separates request IDs, control_seq, shared target/heartbeat seq, run_id, and state_serial/server_id.',['udp-client','concurrency','session-management','heartbeat','retry']],
'UdpControl/debug_ui.py':['Provides a Tk debug panel that selects devices, scans the controller, enables runs, streams coalesced slider targets, and displays axis and endpoint-limit feedback through the UDP SDK. A dedicated executor submits stop requests independently of ordinary ACK waits.',['component','udp-client','event-handler','telemetry','limit-feedback']],
'UdpControl/example_client.py':['Offers a command-line SDK example that queries status by default and uses --move to scan, enable selected axes, and send sinusoidal offsets at approximately 60 Hz. It disables the run in a finally block.',['entry-point','example','udp-client','motion-control']],
'UdpControl/wire.py':['Provides compact UTF-8 JSON encoding and strict datagram decoding for portable clients. Decoding rejects oversized packets, non-object JSON, and non-finite constants; the SDK currently uses encode and handles received JSON itself.',['serialization','validation','udp-protocol']],
'test_udp_protocol.py':['Verifies UDP request deduplication, control-sequence replay protection, ownership and authentication, run isolation, leases, malformed packets, lost-ACK retries, and server-restart feedback filtering. Uses a mocked motor service and loopback sockets.',['test','udp-protocol','retry','session-management','telemetry']],
'test_udp_service.py':['Exercises the UDP SDK, server, and motor service together with an injected fake EtherCAT controller. Covers selected-axis runs, unlimited offsets, heartbeat watchdogs, independent target ordering, fault recovery, and a portable client subprocess.',['test','integration-test','heartbeat','fault-recovery','portable-client']]
};
const symbolSemantics={
'load_config':['Reads and validates endpoint/auth settings from JSON while ignoring unrelated backend configuration fields. Defaults to localhost port 5005 and an empty authentication key.',['configuration','validation','udp-client']],
'UdpControl/__main__.py:main':['Parses client launch arguments, initializes rotating logs, starts the Tk debug window, and reports failures using a Windows message box. Closes the SDK and destroys Tk objects on the UI thread.',['entry-point','logging','component']],
'UdpControl/example_client.py:main':['Queries backend status or, with --move, claims ownership and sends sinusoidal targets for the requested duration. Waits for scan/enable phases and guarantees a disable request after motion.',['entry-point','example','motion-control']],
'encode':['Serializes a packet as compact UTF-8 JSON and disallows NaN and Infinity.',['serialization','json','udp-protocol']],
'decode':['Rejects datagrams over 8192 bytes, non-finite JSON constants, and JSON values that are not objects. Returns the parsed packet dictionary.',['validation','serialization','udp-protocol']],
'UDPError':['Signals rejected, failed, or unacknowledged SDK requests and state-wait failures.',['error-handling','udp-client','type-definition']],
'MotorClient':['Owns the UDP socket, request correlation, session/run identity, feedback state, receiver thread, and independent heartbeat thread. Exposes a portable motor API with retries and best-effort shutdown.',['udp-client','concurrency','session-management','heartbeat']],
'DebugWindow':['Coordinates the Tk controls, motor selection, queued SDK work, dedicated stop executor, and feedback rendering. All controller actions are sent through MotorClient.',['component','event-handler','udp-client','telemetry']],
'ProtocolTests':['Tests server protocol behavior using a mock service and real loopback UDP for retry and restart scenarios.',['test','udp-protocol','retry']],
'ServiceTests':['Builds a real UDP client/server/service stack over FakeMaster-backed EtherCAT controllers and tests complete run lifecycles without hardware.',['test','integration-test','dependency-injection']],
'MotorClient.__init__':['Validates the heartbeat interval and initializes the UDP socket, synchronized state, pending-request map, and receiver/heartbeat daemon threads.',['initialization','concurrency','udp-client']],
'MotorClient._receive':['Receives packets only from the configured peer, correlates ACKs to pending IDs, and accepts newer telemetry only from the current server_id. Ignores stale state_serial values and records socket failures.',['event-handler','telemetry','validation','concurrency']],
'MotorClient.request':['Creates one UUID request ID and encoded packet per operation, then resends that exact datagram until an ACK arrives or retries expire. Adds session/auth data and advances control_seq for adapters, scan, enable, and release.',['retry','udp-protocol','session-management','concurrency']],
'MotorClient.hello':['Serializes ownership handshakes and updates server/session identity from the ACK. A changed server refreshes feedback ordering, while a new owner session resets run state and synchronizes control_seq.',['session-management','reconnection','telemetry']],
'MotorClient.wait_for':['Polls status until an allowed phase appears, raises the backend fault message immediately, and fails when its deadline expires.',['polling','fault-handling','udp-client']],
'MotorClient._heartbeat':['Sends heartbeats only for the current owned run when matching feedback says enabling or enabled. Uses an independent interval thread and records request failures for observation.',['heartbeat','concurrency','run-isolation']],
'MotorClient.close':['Attempts to disable an active run, wait for cleanup, and release ownership before closing the socket and joining both background threads. Failed requests leave the server watchdog responsible for stopping the run.',['cleanup','heartbeat','session-management']],
'DebugWindow.__init__':['Creates the UDP SDK, ordinary and stop executors, event queue, and Tk controls for card selection, per-axis feedback, run parameters, and slider offsets. Registers a 20 ms UI polling loop.',['initialization','component','event-handler']],
'DebugWindow.stop':['Drops any unsent target and schedules disable on the dedicated stop executor when this client owns an active run. Returns worker results/errors to the UI event queue.',['event-handler','motion-control','concurrency']],
'DebugWindow.task':['Allows one ordinary SDK operation at a time and executes it outside the Tk thread. Sends its labeled result or exception back through the event queue.',['concurrency','event-handler','task-queue']],
'DebugWindow.enable':['Requires acknowledged run conditions and selected motors, parses RPM/acceleration values, and queues an SDK enable call for sorted selected axis orders.',['validation','event-handler','motion-control']],
'DebugWindow.set_range':['Validates a finite slider center and positive span, then updates display bounds while suppressing target changes during synchronization.',['validation','component','display-range']],
'DebugWindow.render':['Updates axis positions, enable/fault state, digital inputs, and both endpoint-limit indicators from feedback. Enables controls according to ownership, backend phase, and matching run_id, clearing unsent targets when motion control is unavailable.',['telemetry','limit-feedback','component','run-isolation']],
'DebugWindow.poll':['Drains worker events, displays errors, refreshes controller choices, renders status replies, and marks limit feedback unknown after one second without updates. Coalesces slider targets and otherwise sends hello every 200 ms to refresh observation/idle ownership leases.',['event-handler','polling','telemetry','target-coalescing']],
'ProtocolTests.setUp':['Creates a mocked idle motor service and ephemeral UDP server, then obtains an owner session using a direct hello packet.',['test','fixture','session-management']],
'ProtocolTests.test_owner_auth_and_disable_run_isolation':['Checks exclusive ownership, read-only hello, stale-run disable rejection, matching-run stop, and authentication requirements for a non-loopback bind.',['test','authentication','run-isolation']],
'ProtocolTests.test_release_retry_and_idle_lease_expiry':['Checks idempotent release retries, fresh sessions after release, and replacement ownership after an idle lease expires. Rejects commands carrying the expired token.',['test','retry','session-management']],
'ProtocolTests.test_real_udp_lost_ack_retry_executes_enable_once':['Drops the first enable ACK on a real loopback socket and confirms MotorClient retries successfully while the service executes enable exactly once.',['test','retry','idempotency']],
'ProtocolTests.test_client_reconnects_after_restart_and_ignores_old_server_feedback':['Restarts the UDP server on the same port and verifies the client obtains new server/session identities. Injects old-server telemetry with a high state_serial and confirms it cannot replace current feedback.',['test','reconnection','telemetry']],
'ServiceTests.setUp':['Injects FakeMaster-backed EtherCAT controllers into MotorService, starts an ephemeral UDP server, and connects an owner MotorClient.',['test','fixture','dependency-injection']],
'ServiceTests.test_multiple_runs_selection_unlimited_target_and_no_retrigger':['Checks selected-axis large offsets, deduplication of unchanged targets, verified disable completion, distinct run IDs after re-enable, and rejection of a stale-run disable.',['test','motion-control','run-isolation']],
'ServiceTests.test_heartbeat_timeout_rejects_late_packets':['Suppresses client heartbeats and verifies the watchdog stops the run, rejects late targets, and reports all axes disabled with the heartbeat timeout reason.',['test','heartbeat','watchdog']],
'ServiceTests.test_reorder_heartbeats_do_not_supersede_targets_or_renew_duplicates':['Checks that newer heartbeat seq values do not supersede target ordering, older targets are rejected, and duplicate heartbeats do not renew the lease. An already expired watchdog rejects subsequent heartbeats.',['test','heartbeat','sequence-ordering']],
'ServiceTests.test_socket_failure_and_fault_require_rescan':['Injects a fake EtherCAT communication failure and verifies queue closure, fault state, and enable rejection until rescan. Then closes the UDP server and verifies the recovered run stops with all axes disabled.',['test','fault-recovery','watchdog']],
'ServiceTests.test_separate_portable_client_process_controls_backend_only_over_udp':['Copies only the portable Python client package into a temporary directory and runs it in a separate interpreter with site imports disabled. Checks UDP scan/enable/target/disable behavior and selected-axis results.',['test','portable-client','integration-test']]
};
const methodRanges={
'UdpControl/client.py':{'MotorClient.__init__':[17,39],'MotorClient._receive':[41,72],'MotorClient.request':[74,103],'MotorClient.hello':[105,119],'MotorClient.wait_for':[160,169],'MotorClient._heartbeat':[171,179],'MotorClient.close':[181,198]},
'UdpControl/debug_ui.py':{'DebugWindow.__init__':[12,105],'DebugWindow.stop':[114,125],'DebugWindow.task':[127,136],'DebugWindow.enable':[160,170],'DebugWindow.set_range':[172,185],'DebugWindow.render':[196,255],'DebugWindow.poll':[257,297]},
'test_udp_protocol.py':{'ProtocolTests.setUp':[12,22],'ProtocolTests.test_owner_auth_and_disable_run_isolation':[43,56],'ProtocolTests.test_release_retry_and_idle_lease_expiry':[58,68],'ProtocolTests.test_real_udp_lost_ack_retry_executes_enable_once':[79,99],'ProtocolTests.test_client_reconnects_after_restart_and_ignores_old_server_feedback':[101,125]},
'test_udp_service.py':{'ServiceTests.setUp':[18,27],'ServiceTests.test_multiple_runs_selection_unlimited_target_and_no_retrigger':[52,70],'ServiceTests.test_heartbeat_timeout_rejects_late_packets':[72,82],'ServiceTests.test_reorder_heartbeats_do_not_supersede_targets_or_renew_duplicates':[84,97],'ServiceTests.test_socket_failure_and_fault_require_rescan':[99,116],'ServiceTests.test_separate_portable_client_process_controls_backend_only_over_udp':[118,146]}
};
const nodes=[],edges=[],edgeKeys=new Set();
const addEdge=(source,target,type,weight)=>{const key=`${source}|${target}|${type}`;if(source!==target&&!edgeKeys.has(key)){edgeKeys.add(key);edges.push({source,target,type,direction:'forward',weight});}};
const addNode=(file,type,name,range,semantics)=>{
 const [summary,tags]=semantics;
 const span=range?range[1]-range[0]+1:extraction.results.find(x=>x.path===file).nonEmptyLines;
 const id=type==='file'?`file:${file}`:`${type}:${file}:${name}`;
 const node={id,type,name,filePath:file,summary,tags,complexity:span>200?'complex':span>=50?'moderate':'simple'};
 if(range)node.lineRange=range;
 nodes.push(node);if(type!=='file')addEdge(`file:${file}`,id,'contains',1);
 return id;
};
for(const r of extraction.results){
 addNode(r.path,'file',r.path.split('/').at(-1),null,fileSemantics[r.path]);
 for(const imp of batch.batchImportData[r.path])addEdge(`file:${r.path}`,`file:${imp}`,'imports',.7);
 const exported=new Set((r.exports||[]).map(x=>x.name));
 for(const f of r.functions||[]){if(f.endLine-f.startLine+1<10&&!exported.has(f.name))continue;const id=addNode(r.path,'function',f.name,[f.startLine,f.endLine],symbolSemantics[`${r.path}:${f.name}`]||symbolSemantics[f.name]);if(exported.has(f.name))addEdge(`file:${r.path}`,id,'exports',.8);}
 for(const c of r.classes||[]){if(c.endLine-c.startLine+1<20&&c.methods.length<2&&!exported.has(c.name))continue;const id=addNode(r.path,'class',c.name,[c.startLine,c.endLine],symbolSemantics[c.name]);if(exported.has(c.name))addEdge(`file:${r.path}`,id,'exports',.8);
  for(const m of c.methods){const name=`${c.name}.${m}`,range=methodRanges[r.path]?.[name];if(!range||range[1]-range[0]+1<10)continue;const methodId=addNode(r.path,'function',name,range,symbolSemantics[name]);addEdge(id,methodId,'contains',1);}
 }
}
const allIDs=new Set(nodes.map(n=>n.id));
const importedSymbols={
'UdpControl/__main__.py':{DebugWindow:['UdpControl/debug_ui.py','DebugWindow','class']},
'UdpControl/client.py':{encode:['UdpControl/wire.py','encode','function']},
'UdpControl/debug_ui.py':{MotorClient:['UdpControl/client.py','MotorClient','class']},
'UdpControl/example_client.py':{MotorClient:['UdpControl/client.py','MotorClient','class']},
'test_udp_protocol.py':{MotorClient:['UdpControl/client.py','MotorClient','class'],UDPServer:['udp_server.py','UDPServer','class'],decode:['udp_server.py','decode','function']},
'test_udp_service.py':{MotorClient:['UdpControl/client.py','MotorClient','class'],UDPServer:['udp_server.py','UDPServer','class'],MotorService:['motor_service.py','MotorService','class'],EtherCATController:['core.py','EtherCATController','class'],FakeMaster:['test_platform_control.py','FakeMaster','class']}
};
for(const r of extraction.results){
 for(const call of r.callGraph||[]){
  const caller=nodes.filter(n=>n.type==='function'&&n.filePath===r.path&&n.lineRange[0]<=call.lineNumber&&n.lineRange[1]>=call.lineNumber).sort((a,b)=>(a.lineRange[1]-a.lineRange[0])-(b.lineRange[1]-b.lineRange[0]))[0];
  if(!caller)continue;
  const imported=importedSymbols[r.path]?.[call.callee];
  if(imported){addEdge(caller.id,`${imported[2]}:${imported[0]}:${imported[1]}`,'calls',.8);continue;}
  let candidate=`function:${r.path}:${call.callee}`;
  if(call.callee.startsWith('self.')){
   const owner=caller.name.split('.')[0];candidate=`function:${r.path}:${owner}.${call.callee.slice(5)}`;
  }
  if(allIDs.has(candidate))addEdge(caller.id,candidate,'calls',.8);
 }
}
// Add calls through imported SDK instances only where their receiver identity is explicit.
const sdkCallers={
'UdpControl/__main__.py:main':[],
'UdpControl/example_client.py:main':['MotorClient.hello','MotorClient.request','MotorClient.wait_for'],
'UdpControl/debug_ui.py:DebugWindow.poll':['MotorClient.hello'],
'test_udp_protocol.py:ProtocolTests.test_real_udp_lost_ack_retry_executes_enable_once':['MotorClient.hello'],
'test_udp_protocol.py:ProtocolTests.test_client_reconnects_after_restart_and_ignores_old_server_feedback':['MotorClient.hello','MotorClient.close'],
'test_udp_service.py:ServiceTests.setUp':['MotorClient.hello'],
'test_udp_service.py:ServiceTests.test_multiple_runs_selection_unlimited_target_and_no_retrigger':['MotorClient.request','MotorClient.wait_for'],
'test_udp_service.py:ServiceTests.test_heartbeat_timeout_rejects_late_packets':['MotorClient.request','MotorClient.wait_for'],
'test_udp_service.py:ServiceTests.test_socket_failure_and_fault_require_rescan':['MotorClient.wait_for']
};
for(const [source,targets] of Object.entries(sdkCallers))for(const target of targets)addEdge(`function:${source}`,`function:UdpControl/client.py:${target}`,'calls',.8);
for(const test of ['test_udp_protocol.py','test_udp_service.py'])addEdge('file:UdpControl/client.py',`file:${test}`,'tested_by',.5);
if(edges.filter(x=>x.type==='imports').length!==Object.values(batch.batchImportData).reduce((n,a)=>n+a.length,0))throw Error('Import count mismatch');
const knownIDs=new Set(nodes.map(n=>n.id));
for(const ns of Object.values(batch.neighborMap))for(const n of ns){knownIDs.add(`file:${n.path}`);for(const s of n.symbols){knownIDs.add(`function:${n.path}:${s}`);knownIDs.add(`class:${n.path}:${s}`);}}
for(const imports of Object.values(batch.batchImportData))for(const p of imports)knownIDs.add(`file:${p}`);
for(const edge of edges)if(!knownIDs.has(edge.source)||!knownIDs.has(edge.target))throw Error(`Unknown endpoint: ${JSON.stringify(edge)}`);
const parts=Math.ceil(Math.max(nodes.length/60,edges.length/120));
const files=batch.files.map(x=>x.path).sort(),size=Math.ceil(files.length/parts);
for(let i=0;i<parts;i++){
 const selected=new Set(files.slice(i*size,(i+1)*size));
 const partNodes=nodes.filter(n=>selected.has(n.filePath)),ids=new Set(partNodes.map(n=>n.id));
 const fragment={nodes:partNodes,edges:edges.filter(e=>ids.has(e.source))};
 const path=`${root}/.ua/intermediate/batch-2${parts>1?`-part-${i+1}`:''}.json`;
 fs.writeFileSync(path,JSON.stringify(fragment,null,2)+'\n');
 const reread=JSON.parse(fs.readFileSync(path,'utf8'));console.log(JSON.stringify({path,nodes:reread.nodes.length,edges:reread.edges.length}));
}
console.log(JSON.stringify({totalNodes:nodes.length,totalEdges:edges.length,imports:edges.filter(x=>x.type==='imports').length,parts,filesSkipped:extraction.filesSkipped,warning:'Bundled extraction lists method names but not ranges; method ranges were supplemented from verified source boundaries.'}));
