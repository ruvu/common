<template>
  <b-input-group>
    <b-input-group-addon>Topic name</b-input-group-addon>
    <b-form-input v-model="topicName"
                  placeholder="Please enter a topic"></b-form-input>
  </b-input-group>
</template>

<script>
export default {
  props: {
    initialTopicName: {
      required: true,
      type: String
    },
    messageType: {
      type: String
    }
  },
  data () {
    return {
      topicName: this.initialTopicName
    }
  },
  watch: {
    topicName () {
      this.$emit('updated', this.topicName)
    }
  }
}
</script>
