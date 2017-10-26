<template>
  <div class='canvas-container'>
    <canvas :id='canvasId' v-touch:pan='onPan'></canvas>
    <resize-observer @notify="resizeCanvas" />
  </div>
</template>

<script>
import randomID from 'random-id'

export default {
  props: {
    rate: {
      type: Number,
      default: 10
    }
  },
  data () {
    return {
      canvasId: `canvas-${randomID(5)}`,
      canvas: '',
      ctx: '',
      logic: {
        startPosition: null,
        currentPosition: null,
        publishing: false
      }
    }
  },
  mounted () {
    this.canvas = document.getElementById(this.canvasId)
    this.ctx = this.canvas.getContext('2d')
    this.resizeCanvas()

    var teleop = this
    setInterval(() => {
      if (teleop.logic.publishing) {
        if (!teleop.logic.currentPosition || !teleop.logic.startPosition) {
          return
        }

        var dx = teleop.logic.currentPosition.x - teleop.logic.startPosition.x
        var dy = teleop.logic.currentPosition.y - teleop.logic.startPosition.y

        var py = -dy / teleop.canvas.height
        var px = dx / teleop.canvas.width

        this.$emit('publish', {px: px, py: py})
      }
    }, 1000 / this.rate)
  },
  methods: {
    resizeCanvas () {
      this.canvas.width = this.canvas.parentElement.offsetWidth
      this.canvas.height = this.canvas.parentElement.offsetHeight
    },
    start (x, y) {
      this.logic.startPosition = {x: x, y: y}
      this.logic.publishing = true
      this.redraw()
    },
    move (x, y) {
      if (!this.logic.startPosition) {
        return
      }
      this.logic.currentPosition = {x: x, y: y}
      this.redraw()
    },
    end () {
      if (this.logic.startPosition) {
        this.$emit('publish', {px: 0, py: 0})
      }
      this.logic.startPosition = false
      this.logic.currentPosition = false

      this.logic.publishing = false

      this.redraw()
    },
    redraw () {
      this.clearCanvas()
      this.draw()
    },
    clearCanvas () {
      this.ctx.fillStyle = '#4D4D4D'
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)
    },
    draw () {
      var r, radgrad
      if (this.logic.startPosition) {
        r = 60
        radgrad = this.ctx.createRadialGradient(0, 0, 0, 0, 0, 2 * r)

        this.ctx.save() // save original state
        this.ctx.translate(this.logic.startPosition.x,
                           this.logic.startPosition.y)

        radgrad.addColorStop(0, 'rgba(0,0,0,0)')
        radgrad.addColorStop(0.4, 'rgba(0,0,0,0)')
        radgrad.addColorStop(0.45, 'rgba(0,0,0,1)')
        radgrad.addColorStop(0.55, 'rgba(0,0,0,1)')
        radgrad.addColorStop(0.6, 'rgba(0,0,0,0)')
        radgrad.addColorStop(1, 'rgba(0,0,0,0)')

        // draw shape
        this.ctx.fillStyle = radgrad
        this.ctx.fillRect(-2 * r, -2 * r, 4 * r, 4 * r)

        this.ctx.restore() // restore original state
      }

      if (this.logic.currentPosition) {
        r = 45
        radgrad = this.ctx.createRadialGradient(0, 0, 0, 0, 0, r)

        this.ctx.save() // save original state
        this.ctx.translate(this.logic.currentPosition.x,
                           this.logic.currentPosition.y)

        radgrad.addColorStop(0, 'rgba(0,0,0,1)')
        radgrad.addColorStop(0.9, 'rgba(0,0,0,1)')
        radgrad.addColorStop(1, 'rgba(0,0,0,0)')

        // draw shape
        this.ctx.fillStyle = radgrad
        this.ctx.fillRect(-r, -r, 2 * r, 2 * r)

        this.ctx.restore() // restore original state
      }
    },
    getCenterWRTCanvas (pageCenter) {
      var boundingRect = this.canvas.getBoundingClientRect()
      var x = pageCenter.x - boundingRect.x
      var y = pageCenter.y - boundingRect.y
      return {x: x, y: y}
    },
    onPan (touchType, e) {
      var center = this.getCenterWRTCanvas(e.center)

      switch (touchType) {
        case 'panstart':
          this.start(center.x, center.y)
          break
        case 'panmove':
          this.move(center.x, center.y)
          break
        case 'panend':
          this.end()
          break
      }
    }
  }
}
</script>

<style>
.canvas-container {
  width: 100%;
  height: 100%;
}
canvas {
  touch-action: none;
}
</style>
