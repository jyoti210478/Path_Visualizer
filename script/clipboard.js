
$(document).ready(function(){

  // Alert after 1 second (currently commented out)
  setTimeout(function(){
      // window.alert("THIS PAGE IS UNDER DEVELOPMENT, HOPE YOU LIKE IT !");
  }, 1000);

  // Main function to copy text to clipboard
  $("#copyText").on("click", function(){
      var $temp = $("<textarea>");
      $("body").append($temp);
      var html = $("#fetch-code").html();

      // Replacing HTML special characters with actual symbols
      html = html.replace(/<div>/g, "\n"); // Replace opening div tags with new lines
      html = html.replace(/<\/div>/g, ""); // Remove closing div tags
      html = html.replace(/&lt;/g, "<"); // Replace encoded less-than symbols
      html = html.replace(/&gt;/g, ">"); // Replace encoded greater-than symbols
      html = html.replace(/&nbsp;&nbsp;&nbsp;&nbsp;/g, "    "); // Replace multiple non-breaking spaces with tabs
      html = html.replace(/&nbsp;/g, " "); // Replace single non-breaking spaces with regular spaces
      html = html.replace(/&amp;/g, "&"); // Replace encoded ampersands
      console.log(html);

      // Copy to clipboard
      $temp.val(html).select();
      document.execCommand("copy");
      $temp.remove();

      // Display copy alert
      $(".copy-alert").removeClass("disappear").addClass("appear");
      setTimeout(function(){ 
          $(".copy-alert").removeClass("appear").addClass("disappear");
      }, 1500);
  });

  // Display sorry alert on footer link click
  $(".footer-link-items a, #sorry").on("click", function(){
      $(".sorry-alert").removeClass("disappear").addClass("appear");
      setTimeout(function(){ 
          $(".sorry-alert").removeClass("appear").addClass("disappear");
      }, 3000);
  });

});



// $(document).ready(function(){

//   // Alert after 1 second
//   setTimeout(function(){
//       // window.alert("THIS PAGE IS UNDER DEVELOPMENT, HOPE YOU LIKE IT !");
//   }, 1000);

//   // Main function to copy text to clipboard
//   $("#copyText").on("click", function(){
//       var $temp = $("<textarea>");
//       $("body").append($temp);
//       var html = $("#fetch-code").html();

//       // Replacing HTML special characters with actual symbols
//       html = html.replace(/\<div\>/g, "\n"); // Replace opening div tags with new lines
//       html = html.replace(/\<\/div>/g, ""); // Remove closing div tags
//       html = html.replace(/&lt;/g, "<"); // Replace encoded less-than symbols
//       html = html.replace(/&gt;/g, ">"); // Replace encoded greater-than symbols
//       html = html.replace(/&nbsp;&nbsp;&nbsp;&nbsp;/g, "    "); // Replace multiple non-breaking spaces with tabs
//       html = html.replace(/&nbsp;/g, " "); // Replace single non-breaking spaces with regular spaces
//       html = html.replace(/&amp;/g, "&"); // Replace encoded ampersands
//       console.log(html);

//       // Copy to clipboard
//       $temp.val(html).select();
//       document.execCommand("copy");
//       $temp.remove();

//       // Display copy alert
//       $(".copy-alert").removeClass("disappear").addClass("appear");
//       setTimeout(function(){ 
//           $(".copy-alert").removeClass("appear").addClass("disappear");
//       }, 1500);
//   });

//   // Display sorry alert on footer link click
//   $(".footer-link-items a, #sorry").on("click", function(){
//       $(".sorry-alert").removeClass("disappear").addClass("appear");
//       setTimeout(function(){ 
//           $(".sorry-alert").removeClass("appear").addClass("disappear");
//       }, 3000);
//   });

// });
