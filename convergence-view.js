const NS='http://www.w3.org/2000/svg';
const node=(tag,attrs={},text)=>{const el=document.createElementNS(NS,tag);for(const [key,value]of Object.entries(attrs))el.setAttribute(key,String(value));if(text!=null)el.textContent=text;return el;};
const num=value=>Number.isFinite(value)?value.toExponential(5):'—';
export class ConvergenceView {
  constructor(root){
    this.root=root;
    root.innerHTML='<div class="row"><h2>代价收敛 · 累计历史</h2><label>查看范围<select aria-label="收敛曲线阶段"></select></label></div><p class="small convergence-status" role="status"></p><svg viewBox="0 0 800 220" role="img" aria-label="真实迭代代价累计曲线" style="width:100%;display:block"></svg><p class="mono small convergence-values"></p><p class="small">横轴：累计代价报告序号（悬停查看原始迭代号）；纵轴：原始 cost，非 RMS。线性化不清空历史；阶段切换或迭代号回退时标记分界且不跨段连线，不同目标函数的 cost 不能直接比较。</p>';
    this.select=root.querySelector('select');this.svg=root.querySelector('svg');this.status=root.querySelector('.convergence-status');this.values=root.querySelector('.convergence-values');
    this.select.addEventListener('change',()=>this.draw());this.reset();
  }
  reset(){clearInterval(this.timer);this.series=[];this.current=null;this.reportCount=0;this.select.replaceChildren();const all=document.createElement('option');all.value='all';all.textContent='全部阶段 · 累计历史';this.select.append(all);this.lastUpdate=null;this.started=null;this.ended=null;this.activity='等待求解器事件';this.values.textContent='尚无迭代代价';this.draw();this.updateStatus();}
  updateStatus(){const now=performance.now();this.status.textContent=`${this.activity}${this.started==null?'':` · 求解已用时 ${((this.ended??now)-this.started)/1000|0} 秒`}${this.lastUpdate==null?'':` · 距最近求解事件 ${Math.max(0,((this.ended??now)-this.lastUpdate)/1000)|0} 秒`}${!this.current?' · 尚无 cost 数据，不生成曲线':''}`;}
  event(detail){
    if(detail.kind==='stage' && detail.payload==='solve'){
      this.started=performance.now();this.ended=null;this.activity='求解器准备中';clearInterval(this.timer);this.timer=setInterval(()=>this.updateStatus(),1000);this.updateStatus();return;
    }
    if(!['solveProgress','solveActivity'].includes(detail.kind))return;
    if(this.started==null){this.started=performance.now();this.ended=null;this.timer=setInterval(()=>this.updateStatus(),1000);}
    this.lastUpdate=performance.now();const p=detail.payload||{};
    if(detail.kind==='solveActivity'){
      this.activity=`${detail.stage} / ${p.phase??'求解活动'}${p.completed!=null&&p.total!=null?` · ${p.completed}/${p.total}`:''}${p.iteration!=null?` · iter ${p.iteration}`:''}${p.cancellable===false?' · 当前步骤无取消安全点':''}`;
    }else{
      this.activity=`${detail.stage} · iter ${p.iteration}`;
      if(Number.isFinite(p.cost)&&Number.isFinite(p.iteration)){
        const last=this.current?.points.at(-1);
        if(!this.current||this.current.stage!==detail.stage||(last&&p.iteration<last.iteration)){
          this.current={stage:detail.stage,points:[]};this.series.push(this.current);const option=document.createElement('option');option.value=String(this.series.length-1);option.textContent=`${this.series.length}. ${detail.stage}`;this.select.append(option);
        }
        this.current.points.push({...p,report:++this.reportCount,segment:this.series.length,elapsedMs:detail.elapsedMs});this.draw();
      }
    }
    this.updateStatus();
  }
  stop(state){this.ended=performance.now();clearInterval(this.timer);this.activity=state;this.updateStatus();}
  draw(){
    this.svg.replaceChildren();const groups=this.select.value==='all'?this.series:[this.series[Number(this.select.value)]].filter(Boolean);const points=groups.flatMap(group=>group.points);
    if(!points.length){this.svg.append(node('text',{x:400,y:110,'text-anchor':'middle',fill:'currentColor','font-size':14},'等待真实迭代数据'));return;}
    const first=points[0],last=points.at(-1);
    this.values.textContent=`显示 ${points.length}/${this.reportCount} 次代价报告 · ${groups.length} 段 · 最新 iter ${last.iteration}/${last.max_iterations??'—'} · cost ${num(last.cost)} · damping ${num(last.damping)} · gradient∞ ${num(last.scaled_gradient_inf_norm)} · 接受/拒绝 ${last.accepted_steps??'—'}/${last.rejected_steps??'—'}`;
    let low=Math.min(...points.map(p=>p.cost)),high=Math.max(...points.map(p=>p.cost));const pad=(high-low)*.06||Math.max(Math.abs(high)*.05,1e-6);low-=pad;high+=pad;
    const minX=first.report,maxX=Math.max(minX+1,last.report),x=i=>92+(i-minX)/(maxX-minX)*680,y=v=>185-(v-low)/(high-low)*165;
    for(let i=0;i<=4;i++){const v=low+(high-low)*i/4,py=y(v);this.svg.append(node('line',{x1:92,y1:py,x2:772,y2:py,stroke:'var(--border)'}),node('text',{x:86,y:py+4,'text-anchor':'end',fill:'currentColor','font-size':11},v.toPrecision(8)));}
    this.svg.append(node('text',{x:92,y:210,fill:'currentColor','font-size':12},String(minX)),node('text',{x:772,y:210,'text-anchor':'end',fill:'currentColor','font-size':12},String(last.report)));
    for(const [index,group] of groups.entries()){
      const start=group.points[0];
      if(index>0)this.svg.append(node('line',{x1:x(start.report),y1:20,x2:x(start.report),y2:185,stroke:'var(--muted)','stroke-dasharray':'4 4'}),node('text',{x:x(start.report)+3,y:15,fill:'currentColor','font-size':11},`段 ${start.segment}`));
      this.svg.append(node('polyline',{points:group.points.map(p=>`${x(p.report)},${y(p.cost)}`).join(' '),fill:'none',stroke:'var(--accent)','stroke-width':2}));
      for(const p of group.points){const circle=node('circle',{cx:x(p.report),cy:y(p.cost),r:3,fill:'var(--accent)'});circle.append(node('title',{},`报告 ${p.report} · 段 ${p.segment} ${group.stage} · iter ${p.iteration} · cost ${p.cost} · ${(p.elapsedMs/1000).toFixed(2)} s`));this.svg.append(circle);}
    }
  }
}
