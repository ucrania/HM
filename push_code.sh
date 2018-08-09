git add .
var=$(zenity --entry --text="Insert your comment, it will apear in Github!" --title="Commit comment")
git commit -m $var
git push
zenity --info