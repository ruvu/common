<template>
  <b-input-group>
    <b-input-group-addon v-text="label"></b-input-group-addon>
    <b-form-input v-model="frameId"
                  placeholder="Please enter a frame id"></b-form-input>
  </b-input-group>
</template>

<script>
export default {
  props: {
    initialFrameId: {
      required: true,
      type: String
    },
    label: {
      type: String,
      default: 'Frame name'
    }
  },
  data () {
    return {
      frameId: this.initialFrameId
    }
  },
  watch: {
    frameId () {
      this.$emit('updated', this.frameId)
    }
  }
}
</script>
