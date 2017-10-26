<template>
  <b-input-group>
    <b-input-group-addon>Action name {{messageType}}</b-input-group-addon>
    <b-form-input v-model="actionName"
                  placeholder="Please enter an action"></b-form-input>
  </b-input-group>
</template>

<script>
export default {
  props: {
    initialActionName: {
      required: true,
      type: String
    },
    messageType: {
      type: String
    }
  },
  data () {
    return {
      actionName: this.initialActionName
    }
  },
  watch: {
    actionName () {
      this.$emit('updated', this.actionName)
    }
  }
}
</script>
