const PI = 3.14159265;
const PADDING = 5;

class PolarChart {
  constructor(ctx,x,y,r,num_j,num_samp,colors) {
    this.colors = colors;
    this.ctx = ctx;
    this.x = x;
    this.y = y;
    this.r = r;
    this.num_j = num_j;
    this.num_samp = num_samp;
    this.nticks = 30;
    this.data = [...Array(num_j)].map(e => Array(num_samp));
  }

  draw_bg() {
    this.ctx.beginPath();
    this.ctx.arc(this.x, this.y, this.r - PADDING, 0, 2*PI);
    this.ctx.stroke();

    let irad = (2*PI)/this.nticks;
    for (let i = 0; i < this.nticks; i++) {
      this.ctx.beginPath();
      this.ctx.moveTo(this.x+(0.1*this.r*Math.cos(i*irad)), this.y+(0.1*this.r*Math.sin(i*irad)));
      this.ctx.lineTo(this.x+this.r*Math.cos(i*irad), this.y+this.r*Math.sin(i*irad));
      this.ctx.stroke();
    }
  }

  draw_joint(i) {
    let dr = this.r / this.num_samp;
    this.ctx.beginPath();
    let j = 0;
    while (this.data[i][j] === undefined && j < this.num_samp) {
      j++;
    }

    this.ctx.moveTo(this.x + j*dr*Math.cos(this.data[i][j]), this.y + j*dr*Math.sin(this.data[i][j]));

    for (; j < this.num_samp; j++) {
      let v = this.data[i][j];
      this.ctx.lineTo(this.x + j*dr*Math.cos(v), this.y + j*dr*Math.sin(v));
    }
    this.ctx.stroke();
  }

  add_data(jointvals) {
    for (let i = 0; i < this.num_j; i++) {
      this.data[i].shift();
      this.data[i].push(jointvals[i]);
    }
  }

  render() {
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    ctx.strokeStyle = "#ccc";
    ctx.lineWidth = 0.8;
    this.draw_bg();

    ctx.linewidth = 1.0;
    for (let i = 0; i < this.num_j; i++) {
      ctx.strokeStyle = this.colors[i] || "#000";
      this.draw_joint(i);
    }
  }
}
