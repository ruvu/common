<template>
  <div id="app">
      <b-navbar type="light" variant="light" :sticky="true" fixed="top">
      <b-navbar-brand href="#">RUVU</b-navbar-brand>

      <b-nav is-nav-bar>
        <router-link tag="li" to="/" exact>
          <a class="nav-link">CmdVelTeleop</a>
        </router-link>
      </b-nav>

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
  padding-top: 5rem;
}
</style>
