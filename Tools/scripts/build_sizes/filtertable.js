//Modified from https://www.w3schools.com/howto/howto_js_filter_table.asp 
function searchFunc(build) {
  // Declare variables
  var input, filter, table, tr, td, i;
  input = document.getElementById(`search_${build}`);
  filter = input.value.toUpperCase();
  table = document.getElementById(`table_${build}`);
  tr = table.getElementsByTagName("tr");

  // Loop through all table rows, and hide those who don't match the search query
  for (i = 0; i < tr.length; i++) {
    td = tr[i].getElementsByTagName("td");
    if (td[0]) {
      if (contained(td, filter)) {
        tr[i].style.display = "";
      } else {
        tr[i].style.display = "none";
      }
    }
  }
}

function contained(td, filter) {
    ret = 0;
    for (i = 0; i < td.length; i++) {
        txtValue = td[i].textContent || td[i].innerText;
        ret += txtValue.toUpperCase().indexOf(filter) > -1;
    }
    return ret > 0;
}
