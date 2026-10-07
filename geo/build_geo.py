import json, math
LAT0, LON0 = 43.71, -79.345
KX = 111320*math.cos(math.radians(LAT0)); KY = 110540
def P(lat,lon): return ((lon-LON0)*KX, -(lat-LAT0)*KY)   # metres, y down (screen)
def dist(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])

def rdp(pts, eps):
    if len(pts)<3: return pts
    a,b=pts[0],pts[-1]; dx,dy=b[0]-a[0],b[1]-a[1]; L=math.hypot(dx,dy) or 1e-9
    dmax,idx=0,0
    for i in range(1,len(pts)-1):
        p=pts[i]; d=abs(dy*p[0]-dx*p[1]+b[0]*a[1]-b[1]*a[0])/L
        if d>dmax: dmax,idx=d,i
    if dmax>eps: return rdp(pts[:idx+1],eps)[:-1]+rdp(pts[idx:],eps)
    return [a,b]

# ---------- corridor centreline: midpoint of the two OSM carriageways ----------
# Built only from OpenStreetMap (ODbL). Each carriageway is chained way-to-way by
# shared node ids; the centreline is the midpoint between a northbound point and
# the nearest point on the southbound carriageway.
def chain_carriageway(ways, northward):
    bystart={}
    for w in ways: bystart.setdefault(w['nodes'][0],[]).append(w)
    def follow(start):
        path=[start]; seen={start['id']}
        while True:
            nxt=[w for w in bystart.get(path[-1]['nodes'][-1],[]) if w['id'] not in seen]
            if not nxt: break
            w=max(nxt,key=lambda w:(w['geometry'][-1]['lat']-w['geometry'][0]['lat'])*(1 if northward else -1))
            path.append(w); seen.add(w['id'])
        return [P(g['lat'],g['lon']) for w in path for g in w['geometry']]
    starts=[w for w in ways if (w['geometry'][0]['lat']<43.662 if northward else w['geometry'][0]['lat']>43.755)]
    best=max((follow(w) for w in starts), key=lambda pts: sum(dist(pts[i-1],pts[i]) for i in range(1,len(pts))))
    return best
_W=[w for w in json.load(open('geo/dvp_ways.json'))['elements'] if w.get('geometry') and w.get('nodes')]
_nb=chain_carriageway(_W, True); _sb=chain_carriageway(_W, False)
def _nearest_on(poly, p):
    best=(1e18,p)
    for i in range(1,len(poly)):
        a,b=poly[i-1],poly[i]; dx,dy=b[0]-a[0],b[1]-a[1]; L2=dx*dx+dy*dy or 1e-9
        t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/L2)); q=(a[0]+t*dx,a[1]+t*dy); d=dist(q,p)
        if d<best[0]: best=(d,q)
    return best
route=[]
for p in _nb:
    d,q=_nearest_on(_sb,p)
    route.append(((p[0]+q[0])/2,(p[1]+q[1])/2) if d<120 else p)
# densify-free de-dup
route=[route[0]]+[route[i] for i in range(1,len(route)) if dist(route[i],route[i-1])>1]
cum=[0]
for i in range(1,len(route)): cum.append(cum[-1]+dist(route[i-1],route[i]))
def project(pt):
    best=(1e18,0)
    for i in range(1,len(route)):
        a,b=route[i-1],route[i]; dx,dy=b[0]-a[0],b[1]-a[1]; L2=dx*dx+dy*dy or 1e-9
        t=max(0,min(1,((pt[0]-a[0])*dx+(pt[1]-a[1])*dy)/L2))
        q=(a[0]+t*dx,a[1]+t*dy); d=dist(q,pt)
        if d<best[0]: best=(d,cum[i-1]+t*math.sqrt(L2))
    return best
# anchors: model x (km)  ->  OSM exit node
J=json.load(open('geo/junctions.json'))['elements']
def node(ref):
    pts=[P(e['lat'],e['lon']) for e in J if e.get('tags',{}).get('ref')==ref]
    return pts
anchors=[(0.0, route[0])]
for x,refs in [(0.8,['1']),(3.8,['3']),(7.0,['7A','7B']),(10.0,['10']),(10.7,['11']),(11.8,['12A','12B']),(14.0,['14'])]:
    ds=[]
    for r in refs:
        for p in node(r):
            d,s=project(p)
            if d<250: ds.append(s)
    if ds: anchors.append((x, None, sum(ds)/len(ds)))
    else: print('!! no anchor for',x,refs)
# 401 crossing: where the route meets the 401 exit nodes around lat 43.7655
d401,s401=project(P(43.7656,-79.3368))
anchors=[(0.0,None,project(anchors[0][1])[1])]+[a for a in anchors[1:]]+[(15.0,None,s401)]
anchors.sort()
print('anchors (model km -> route m):')
for a in anchors: print('  %5.1f  %7.0f'%(a[0],a[2]))
# sanity: monotone
assert all(anchors[i][2]<anchors[i+1][2] for i in range(len(anchors)-1)), 'non-monotone anchors'
def route_pt(s):
    for i in range(1,len(route)):
        if cum[i]>=s:
            t=(s-cum[i-1])/max(1e-9,cum[i]-cum[i-1]); a,b=route[i-1],route[i]
            return (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
    return route[-1]
def x_to_s(x):
    for i in range(1,len(anchors)):
        if x<=anchors[i][0]:
            a,b=anchors[i-1],anchors[i]; t=(x-a[0])/(b[0]-a[0]); return a[2]+t*(b[2]-a[2])
    return anchors[-1][2]
# sample the centreline every 25 m of MODEL distance -> lookup table
N=601
line=[]
for k in range(N):
    x=15.0*k/(N-1); p=route_pt(x_to_s(x)); line.append([round(p[0],1),round(p[1],1)])
# ---------- carriageways (both) for drawing ----------
W=json.load(open('geo/dvp_ways.json'))['elements']
roads=[]
for w in W:
    pts=[P(g['lat'],g['lon']) for g in w.get('geometry',[])]
    pts=rdp(pts,3)
    roads.append([[round(a,1),round(b,1)] for a,b in pts])
# ---------- river ----------
RV=json.load(open('geo/river.json'))['elements']
river=[]
for w in RV:
    pts=[P(g['lat'],g['lon']) for g in w.get('geometry',[])]
    pts=rdp(pts,6)
    if len(pts)>1: river.append([[round(a),round(b)] for a,b in pts])
# ---------- arterials ----------
A=json.load(open('geo/arterials.json'))['elements']
art=[]; names={}
for w in A:
    t=w.get('tags',{}); hw=t.get('highway'); nm=t.get('name','')
    if hw=='motorway' and nm=='Don Valley Parkway': continue
    pts=[P(g['lat'],g['lon']) for g in w.get('geometry',[])]
    pts=rdp(pts,10)
    if len(pts)<2: continue
    cls={'motorway':2,'trunk':1,'primary':0}[hw]
    art.append({'c':cls,'p':[[round(a),round(b)] for a,b in pts]})
    if nm: names.setdefault(nm,[]).extend(pts)
# label anchors: a representative point for important names
want=['Bloor Street East','Danforth Avenue','Eglinton Avenue East','Lawrence Avenue East','York Mills Road',
      'Don Mills Road','Bayview Avenue','Gardiner Expressway','Highway 401','Queen Street East','Lake Shore Boulevard East','Yonge Street','Victoria Park Avenue']
labels=[]
for nm in want:
    pts=names.get(nm) or names.get(nm.replace('Street','St'))
    if not pts: continue
    # choose the point closest to the corridor, nudged 700 m off it
    best=min(pts,key=lambda p:min(dist(p,(l[0],l[1])) for l in line[::10]))
    labels.append({'t':nm.replace(' Avenue',' Ave').replace(' Street',' St').replace(' Road',' Rd').replace(' Boulevard',' Blvd').replace(' East',' E'),'p':[round(best[0]),round(best[1])]})
xs=[p[0] for p in line]; ys=[p[1] for p in line]
out={'line':line,'roads':roads,'river':river,'art':art,'labels':labels,
     'bbox':[min(xs),min(ys),max(xs),max(ys)],'origin':[LAT0,LON0]}
s=json.dumps(out,separators=(',',':'))
open('src/geo.js','w').write('/* Geometry: derived from OpenStreetMap data (c) OpenStreetMap contributors, ODbL 1.0. Local metres around %.2f,%.3f; y increases southward. */\nconst GEO=%s;\n'%(LAT0,LON0,s))
print('line pts',len(line),'roads',len(roads),'river',len(river),'arterials',len(art),'labels',[l['t'] for l in labels])
print('geo.js size %.1f KB'%(len(s)/1024))

# ---------- city texture: secondary/tertiary streets, rail, green space ----------
def lines_from(fn, eps, keep=None):
    out=[]
    for w in json.load(open(fn))['elements']:
        if keep and not keep(w.get('tags',{})): continue
        g=w.get('geometry') or []
        pts=rdp([P(q['lat'],q['lon']) for q in g], eps)
        if len(pts)>1: out.append([[round(a),round(b)] for a,b in pts])
    return out
sec  = lines_from('geo/secondary.json', 14)
rail = lines_from('geo/rail.json', 12, keep=lambda t: t.get('service') not in ('yard','siding','spur'))
green=[]
for w in json.load(open('geo/parks.json'))['elements']:
    g=w.get('geometry') or []
    if len(g)<4: continue
    raw=[P(q['lat'],q['lon']) for q in g]
    h=len(raw)//2
    pts=rdp(raw[:h+1],18)[:-1]+rdp(raw[h:],18)   # split ring: RDP degenerates when ends coincide
    # drop slivers
    if len(pts)<4: continue
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    if (max(xs)-min(xs))*(max(ys)-min(ys)) < 20000: continue
    green.append([[round(a),round(b)] for a,b in pts])
out.update({'sec':sec,'rail':rail,'green':green})
s=json.dumps(out,separators=(',',':'))
open('src/geo.js','w').write('/* Geometry: derived from OpenStreetMap data (c) OpenStreetMap contributors, ODbL 1.0. Local metres around %.2f,%.3f; y increases southward. */\nconst GEO=%s;\n'%(LAT0,LON0,s))
print('sec',len(sec),'rail',len(rail),'green',len(green),' geo.js %.1f KB'%(len(s)/1024))
