import fs from 'node:fs';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { execFileSync } from 'node:child_process';

const root = process.cwd();
const ua = path.join(root, '.ua');
const read = p => JSON.parse(fs.readFileSync(path.join(ua, p), 'utf8'));
const graph = read('intermediate/assembled-graph.json');
const scan = read('intermediate/scan-result.json');
const layersInput = read('intermediate/layers.json');
const tourInput = read('intermediate/tour.json');
const layers = Array.isArray(layersInput) ? layersInput : layersInput.layers;
const tour = Array.isArray(tourInput) ? tourInput : tourInput.steps;
const ids = new Set(graph.nodes.map(n => n.id));
const levelTypes = new Set(['file','config','document','service','pipeline','resource','schema','table','endpoint']);
const fileNodes = graph.nodes.filter(n => levelTypes.has(n.type));
const assigned = new Set();
for (const layer of layers) {
 if (!layer.id || !layer.name || !layer.description || !Array.isArray(layer.nodeIds)) throw Error('Malformed layer');
 for (const id of layer.nodeIds) {
  if (!ids.has(id) || assigned.has(id)) throw Error(`Invalid/duplicate layer reference: ${id}`);
  assigned.add(id);
 }
}
for (const n of fileNodes) if (!assigned.has(n.id)) throw Error(`Missing layer for ${n.id}`);
for (const f of scan.files) if (!fileNodes.some(n => n.filePath === f.path)) throw Error(`Missing file ${f.path}`);
if (fileNodes.length !== scan.totalFiles) throw Error('File coverage mismatch');
tour.sort((a,b) => a.order-b.order);
for (let i=0;i<tour.length;i++) {
 const s=tour[i];
 if (s.order!==i+1 || !s.title || !s.description || !s.nodeIds?.length || s.nodeIds.some(id=>!ids.has(id))) throw Error('Invalid tour');
}
if(tour.length<5 || tour.length>15) throw Error('Invalid tour length');
const commit=execFileSync('git',['rev-parse','HEAD'],{encoding:'utf8'}).trim();
const full={version:'1.0.0',kind:'codebase',project:{name:scan.name,languages:scan.languages,frameworks:scan.frameworks,description:scan.description,analyzedAt:new Date().toISOString(),gitCommitHash:commit},nodes:graph.nodes,edges:graph.edges,layers,tour};
const schemaPath='C:/Users/Administrator/.understand-anything/repo/understand-anything-plugin/packages/core/dist/schema.js';
const {KnowledgeGraphSchema}=await import(pathToFileURL(schemaPath).href);
const parsed=KnowledgeGraphSchema.safeParse(full);
if(!parsed.success) throw Error(JSON.stringify(parsed.error.issues));
const counts=items=>items.reduce((a,x)=>(a[x.type]=(a[x.type]||0)+1,a),{});
const byCategory=scan.files.reduce((a,x)=>(a[x.fileCategory]=(a[x.fileCategory]||0)+1,a),{});
const notes=['Batch/requirements formats lack specialized structural parsers; their complete source was read to supply file semantics.','Bundled class extraction lists method names without line ranges; significant method positions were verified against source.','Static calls are source-grounded references, not proof of runtime execution or hardware behavior.','Legacy UE three-axis packets are incompatible with the current v1 UDP RPC protocol.'];
fs.writeFileSync(path.join(ua,'intermediate/assembled-graph.json'),JSON.stringify(full,null,2));
fs.writeFileSync(path.join(ua,'intermediate/final-summary.json'),JSON.stringify({files:scan.totalFiles,byCategory,nodeTypes:counts(full.nodes),edgeTypes:counts(full.edges),layers:layers.map(x=>x.name),tourSteps:tour.length,notes},null,2));
console.log(`Schema and coverage passed: ${scan.totalFiles} files, ${full.nodes.length} nodes, ${full.edges.length} edges, ${layers.length} layers, ${tour.length} tour steps.`);
