import fs from 'node:fs';
import path from 'node:path';
try {
 const input=JSON.parse(fs.readFileSync(process.argv[2],'utf8').replace(/^\uFEFF/,''));
 const nodes=input.fileNodes,byID=new Map(nodes.map(n=>[n.id,n]));
 const imports=input.importEdges.filter(e=>byID.has(e.source)&&byID.has(e.target));
 const all=input.allEdges.filter(e=>byID.has(e.source)&&byID.has(e.target));
 const paths=nodes.map(n=>n.filePath.replaceAll('\\','/'));
 let prefix=paths[0].split('/').slice(0,-1);
 for(const p of paths){const segments=p.split('/').slice(0,-1);let length=0;while(length<prefix.length&&length<segments.length&&prefix[length]===segments[length])length++;prefix=prefix.slice(0,length);}
 const commonPrefix=prefix.length?prefix.join('/')+'/':'';
 const flat=paths.every(p=>!p.slice(commonPrefix.length).includes('/'));
 const filePattern=n=>{
  const p=n.filePath.toLowerCase(),name=path.posix.basename(p);
  if(/(?:^|\/)test_[^/]+\.py$|\.(?:test|spec)\.|_test\.go$|test\.java$|_spec\.rb$|tests\.cs$/.test(p))return 'test';
  if(n.type==='config'||/\.config\.|\.toml$|\.ya?ml$/.test(p))return 'config';
  if(/\.d\.ts$|\.(?:graphql|gql|proto)$/.test(p))return 'types';
  if(/(?:^|\/)(?:__init__|__main__|manage|index)\.(?:py|ts|js)$/.test(p))return 'entry';
  if(n.type==='document'||/\.(?:md|rst)$/.test(p))return 'documentation';
  if(/dockerfile|docker-compose|\.tf(?:vars)?$|(?:^|\/)makefile$/.test(p))return 'infrastructure';
  if(/\.github\/workflows\/|\.gitlab-ci\.yml$|jenkinsfile$/.test(p))return 'ci-cd';
  if(/\.sql$/.test(p))return 'data';
  return path.posix.extname(name).slice(1)||'root';
 };
 const directoryGroups={},nodeTypeGroups={},idGroup=new Map(),filePatternMatches={};
 for(const n of nodes){const p=n.filePath.replaceAll('\\','/').slice(commonPrefix.length);const group=flat?filePattern(n):(p.includes('/')?p.split('/')[0]:'root');(directoryGroups[group]??=[]).push(n.id);(nodeTypeGroups[n.type]??=[]).push(n.id);idGroup.set(n.id,group);filePatternMatches[n.id]=filePattern(n);}
 const fileFanIn=Object.fromEntries(nodes.map(n=>[n.id,0])),fileFanOut={...fileFanIn},adjacency=Object.fromEntries(nodes.map(n=>[n.id,[]]));
 const inter=new Map(),groupImports={},density={};
 for(const group of Object.keys(directoryGroups)){groupImports[group]={importsFrom:new Set(),importedBy:new Set()};density[group]={internalEdges:0,totalEdges:0,density:0};}
 for(const e of imports){fileFanOut[e.source]++;fileFanIn[e.target]++;adjacency[e.source].push(e.target);const a=idGroup.get(e.source),b=idGroup.get(e.target);inter.set(`${a}|${b}`,(inter.get(`${a}|${b}`)||0)+1);groupImports[a].importsFrom.add(b);groupImports[b].importedBy.add(a);density[a].totalEdges++;if(a===b)density[a].internalEdges++;else density[b].totalEdges++;}
 for(const d of Object.values(density))d.density=d.totalEdges?d.internalEdges/d.totalEdges:0;
 for(const g of Object.values(groupImports)){g.importsFrom=[...g.importsFrom].sort();g.importedBy=[...g.importedBy].sort();}
 const categories=new Map(),nonCodeConnections=[];
 for(const e of all){const from=byID.get(e.source),to=byID.get(e.target);const key=`${from.type}|${to.type}|${e.type}`;categories.set(key,(categories.get(key)||0)+1);if(from.type!=='file'&&to.type==='file'||from.type==='file'&&to.type!=='file')nonCodeConnections.push(e);}
 const patterns={api:['routes','api','controllers','endpoints','handlers','serializers','controller','routers','blueprints'],service:['services','core','lib','domain','logic','internal','signals','composables','mailers','jobs','channels'],data:['models','db','data','persistence','repository','entities','migrations','entity','sql','database','schema'],ui:['components','views','pages','ui','layouts','screens'],middleware:['middleware','plugins','interceptors','guards'],utility:['utils','helpers','common','shared','tools','pkg','templatetags'],config:['config','constants','env','settings','management','commands'],test:['__tests__','test','tests','spec','specs'],types:['types','interfaces','schemas','contracts','dtos','dto','request','response'],hooks:['hooks'],state:['store','state','reducers','actions','slices'],assets:['assets','static','public'],entry:['cmd','bin'],documentation:['docs','documentation','wiki'],infrastructure:['deploy','deployment','infra','infrastructure','k8s','kubernetes','helm','charts','terraform','tf','docker'],'ci-cd':['.github','.gitlab','.circleci']};
 const patternMatches={};for(const g of Object.keys(directoryGroups)){const match=Object.entries(patterns).find(([,names])=>names.includes(g.toLowerCase()));if(match)patternMatches[g]=match[0];}
 const infra=nodes.filter(n=>['service','resource','pipeline'].includes(n.type)||['infrastructure','ci-cd'].includes(filePattern(n)));
 const deploymentTopology={hasDockerfile:paths.some(p=>/dockerfile/i.test(p)),hasCompose:paths.some(p=>/docker-compose/i.test(p)),hasK8s:paths.some(p=>/(?:^|\/)(?:k8s|kubernetes|helm)\//i.test(p)),hasTerraform:paths.some(p=>/\.tf(?:vars)?$/i.test(p)),hasCI:nodes.some(n=>n.type==='pipeline'||filePattern(n)==='ci-cd'),infraFiles:infra.map(n=>n.filePath),chains:all.filter(e=>infra.some(n=>n.id===e.source)&&infra.some(n=>n.id===e.target)),environments:paths.filter(p=>/(?:dockerfile|docker-compose).*(?:dev|prod|staging)/i.test(p))};
 const dataPipeline={schemaFiles:nodes.filter(n=>['schema','table'].includes(n.type)||/\.(?:sql|graphql|proto|prisma)$/.test(n.filePath)).map(n=>n.filePath),migrationFiles:paths.filter(p=>/(?:^|\/)migrations?\//.test(p)),dataModelFiles:nodes.filter(n=>n.tags?.includes('data-model')).map(n=>n.filePath),apiHandlerFiles:nodes.filter(n=>n.tags?.includes('api-handler')).map(n=>n.filePath)};
 const documented=new Set(nodes.filter(n=>n.type==='document').map(n=>idGroup.get(n.id)));
 for(const e of all)if(e.type==='documents'&&byID.get(e.source).type==='document')documented.add(idGroup.get(e.target));
 const groups=Object.keys(directoryGroups),docCoverage={groupsWithDocs:documented.size,totalGroups:groups.length,coverageRatio:documented.size/groups.length,undocumentedGroups:groups.filter(g=>!documented.has(g)),readmeGroups:nodes.filter(n=>/(?:^|\/)readme\.md$/i.test(n.filePath)).map(n=>idGroup.get(n.id))};
 const dependencyDirection=[],seen=new Set();for(const [key,count]of inter){const [a,b]=key.split('|');if(a===b||seen.has([a,b].sort().join('|')))continue;seen.add([a,b].sort().join('|'));const inverse=inter.get(`${b}|${a}`)||0;if(count!==inverse)dependencyDirection.push({dependent:count>inverse?a:b,dependsOn:count>inverse?b:a,dominantImports:Math.max(count,inverse),reverseImports:Math.min(count,inverse)});}
 const result={scriptCompleted:true,commonPrefix,flatStructure:flat,directoryGroups,nodeTypeGroups,adjacency,groupImports,crossCategoryEdges:[...categories].map(([key,count])=>{const[fromType,toType,edgeType]=key.split('|');return{fromType,toType,edgeType,count};}),nonCodeConnections,interGroupImports:[...inter].map(([key,count])=>{const[from,to]=key.split('|');return{from,to,count};}),intraGroupDensity:density,patternMatches,filePatternMatches,deploymentTopology,dataPipeline,docCoverage,dependencyDirection,fileStats:{totalFileNodes:nodes.length,filesPerGroup:Object.fromEntries(groups.map(g=>[g,directoryGroups[g].length])),nodeTypeCounts:Object.fromEntries(Object.entries(nodeTypeGroups).map(([type,ids])=>[type,ids.length]))},fileFanIn,fileFanOut};
 fs.writeFileSync(process.argv[3],JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify({fileNodes:nodes.length,imports:imports.length,directoryGroups:result.fileStats.filesPerGroup,nodeTypes:result.fileStats.nodeTypeCounts}));
}catch(error){console.error(error.stack);process.exit(1);}
