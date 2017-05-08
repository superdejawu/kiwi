module.exports = function(grunt) {

  grunt.initConfig({

    pkg: grunt.file.readJSON('package.json'),

    coffee: {
      compile: {
        files: {
          'src/Main.js': 'src/*.coffee'
        }
      }
    },

    concat: {
      options: {
        separator: "\n\n"
      },
      dist: {
        src: [
          'src/*.js',
          'src/lib/*.js'
        ],
        dest: '<%= pkg.name.replace(".js", "") %>.js'
      }
    },

    uglify: {
      options: {
        banner: '/*! <%= pkg.name.replace(".js", "") %> <%= grunt.template.today("dd-mm-yyyy") %> */\n'
      },
      dist: {
        files: {
          '<%= pkg.name.replace(".js", "") %>.min.js': ['<%= concat.dist.dest %>']
        }
      }
    },

    qunit: {
      files: ['test/*.html']
    },

    jshint: {
      files: ['KiwiWebClient.js'],
      options: {
        globals: {
          console: true,
          module: true,
          document: true
        },
        jshintrc: '.jshintrc'
      }
    },

    watch: {
      files: ['src/*'],
      tasks: ['coffee', 'concat', 'uglify']
    }

  });

  grunt.loadNpmTasks('grunt-contrib-coffee');
  grunt.loadNpmTasks('grunt-contrib-uglify');
  grunt.loadNpmTasks('grunt-contrib-jshint');
  grunt.loadNpmTasks('grunt-contrib-qunit');
  grunt.loadNpmTasks('grunt-contrib-watch');
  grunt.loadNpmTasks('grunt-contrib-concat');

  grunt.registerTask('test', ['qunit']);
  grunt.registerTask('default', ['coffee', 'concat', 'uglify']);
  grunt.registerTask('dist', ['coffee', 'concat', 'qunit', 'uglify']);

};
