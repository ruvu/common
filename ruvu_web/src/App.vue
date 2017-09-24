<template>
  <div id="app">
    <b-navbar toggleable="md"  type="light" variant="light" :sticky="true" fixed="top">
      <b-nav-toggle target="nav_collapse"></b-nav-toggle>

      <b-collapse is-nav id="nav_collapse">
        <b-nav is-nav-bar>
          <router-link tag="li" to="/" exact>
            <a class="nav-link">Twist teleop</a>
          </router-link>
          <router-link tag="li" to="/text_to_speech" exact>
            <a class="nav-link">Text to speech</a>
          </router-link>
        </b-nav>
      </b-collapse>

      <!-- Right aligned nav items -->
      <b-nav is-nav-bar class="ml-auto">
        <connection-status-button></connection-status-button>
      </b-nav>
    </b-navbar>

    <div class="container" id="main">
      <router-view></router-view>
    </div>
  </div>
</template>

<script>
import ConnectionStatusButton from '@/components/ros/ConnectionStatusButton'

export default {
  components: {
    ConnectionStatusButton
  },
  name: 'app'
}
</script>

<style>
#main {
  padding-top: 20px
}
</style>
